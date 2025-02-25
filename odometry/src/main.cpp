#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <math.h>

// Micro-ROS headers
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int64_multi_array.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

// ------------------------------
// Hardware and Robot Configuration Constants
// ------------------------------

// PID constants - these may need tuning for your specific robot
#define KP 3.5
#define KI 0.3
#define KD 1.0

// Expected wheel ticks per timer interval based on commanded velocity
int32_t expected_ticks_left = 0;
int32_t expected_ticks_right = 0;

// Variables for PID control
float integral[4] = {0.0, 0.0, 0.0, 0.0};
float prev_error[4] = {0.0, 0.0, 0.0, 0.0};
int pwm_offsets[4] = {0, 0, 0, 0};

// Motor indices
#define RIGHT_FRONT 0
#define LEFT_FRONT  1
#define LEFT_REAR   2
#define RIGHT_REAR  3

// Motor Pin Definitions
#define RIGHT_FRONT_ENC_A 7
#define RIGHT_FRONT_ENC_B 6
#define RIGHT_FRONT_MOTOR_DIR 23
#define RIGHT_FRONT_PWM 2

#define LEFT_FRONT_ENC_A 8
#define LEFT_FRONT_ENC_B 9
#define LEFT_FRONT_MOTOR_DIR 22
#define LEFT_FRONT_PWM 3

#define LEFT_REAR_ENC_A 32
#define LEFT_REAR_ENC_B 33
#define LEFT_REAR_MOTOR_DIR 40
#define LEFT_REAR_PWM 4

#define RIGHT_REAR_ENC_A 37
#define RIGHT_REAR_ENC_B 36
#define RIGHT_REAR_MOTOR_DIR 41
#define RIGHT_REAR_PWM 5

// PWM resolution (8-bit)
#define PWM_RESOLUTION 8

// Robot physical parameters
#define WHEEL_RADIUS 0.33          // meters
#define WHEEL_SEPARATION 0.85     // meters
#define ENCODER_COUNTS_PER_REV 1700
#define MAX_RPM 300

// Compute velocity limits
#define MAX_LINEAR_VELOCITY (WHEEL_RADIUS * 2 * PI * MAX_RPM / 60)
#define MAX_ANGULAR_VELOCITY (MAX_LINEAR_VELOCITY / 0.080)  // TURNING_RADIUS assumed = 0.080
#define MIN_LINEAR_VELOCITY (-MAX_LINEAR_VELOCITY)
#define MIN_ANGULAR_VELOCITY (-MAX_ANGULAR_VELOCITY)

// ------------------------------
// Global Variables for Motor Control & Odometry
// ------------------------------
volatile int32_t encoder_data[4] = {0, 0, 0, 0};
int32_t last_encoder_data[4] = {0, 0, 0, 0};

float x_pos = 0.0;
float y_pos = 0.0;
float theta = 0.0;

// Global variable to hold desired velocities [linear, angular]
float goal_velocity[2] = {0.0, 0.0};

// ------------------------------
// Micro-ROS Globals
// ------------------------------
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t odom_publisher;
rcl_publisher_t tf_publisher;
nav_msgs__msg__Odometry odom_msg;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped transform_msg;

// Subscription for cmd_vel
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

// Publisher for debugging: Publish motor PWM values on "chatter"
rcl_publisher_t chatter_publisher;
std_msgs__msg__String chatter_msg;

// NEW: Publisher for individual wheel odometry ("wheels_spin")
rcl_publisher_t wheels_spin_publisher;
std_msgs__msg__Int64MultiArray wheels_spin_msg;

// ------------------------------
// Connection state for micro-ROS
// ------------------------------
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// ------------------------------
// Helper Macros
// ------------------------------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis(); } \
  if (uxr_millis() - init > (MS)) { X; init = uxr_millis(); } \
} while (0)

// ------------------------------
// Encoder ISR Functions
// ------------------------------
void MotorISR_RIGHT_FRONT() {
  int b = digitalRead(RIGHT_FRONT_ENC_B);
  if (digitalRead(RIGHT_FRONT_ENC_A))
    b ? encoder_data[RIGHT_FRONT]-- : encoder_data[RIGHT_FRONT]++;
  else
    b ? encoder_data[RIGHT_FRONT]++ : encoder_data[RIGHT_FRONT]--;
}

void MotorISR_LEFT_FRONT() {
  int b = digitalRead(LEFT_FRONT_ENC_B);
  if (digitalRead(LEFT_FRONT_ENC_A))
    b ? encoder_data[LEFT_FRONT]-- : encoder_data[LEFT_FRONT]++;
  else
    b ? encoder_data[LEFT_FRONT]++ : encoder_data[LEFT_FRONT]--;
}

void MotorISR_LEFT_REAR() {
  int b = digitalRead(LEFT_REAR_ENC_B);
  if (digitalRead(LEFT_REAR_ENC_A))
    b ? encoder_data[LEFT_REAR]-- : encoder_data[LEFT_REAR]++;
  else
    b ? encoder_data[LEFT_REAR]++ : encoder_data[LEFT_REAR]--;
}

void MotorISR_RIGHT_REAR() {
  int b = digitalRead(RIGHT_REAR_ENC_B);
  if (digitalRead(RIGHT_REAR_ENC_A))
    b ? encoder_data[RIGHT_REAR]-- : encoder_data[RIGHT_REAR]++;
  else
    b ? encoder_data[RIGHT_REAR]++ : encoder_data[RIGHT_REAR]--;
}

// ------------------------------
// Motor PWM Control Function
// ------------------------------
void MOTOR_PWM_CONTROL(int motor, int pwm) {
  if (pwm > 255) pwm = 255;
  else if (pwm < -255) pwm = -255;
  
  if (motor == LEFT_FRONT) {
    if (pwm >= 0) {
      digitalWrite(LEFT_FRONT_MOTOR_DIR, LOW);
      analogWrite(LEFT_FRONT_PWM, pwm);
    } else {
      digitalWrite(LEFT_FRONT_MOTOR_DIR, HIGH);
      analogWrite(LEFT_FRONT_PWM, abs(pwm));
    }
  }
  else if (motor == RIGHT_FRONT) {
    if (pwm >= 0) {
      digitalWrite(RIGHT_FRONT_MOTOR_DIR, HIGH);
      analogWrite(RIGHT_FRONT_PWM, pwm);
    } else {
      digitalWrite(RIGHT_FRONT_MOTOR_DIR, LOW);
      analogWrite(RIGHT_FRONT_PWM, abs(pwm));
    }
  }
  else if (motor == LEFT_REAR) {
    if (pwm >= 0) {
      digitalWrite(LEFT_REAR_MOTOR_DIR, LOW);
      analogWrite(LEFT_REAR_PWM, pwm);
    } else {
      digitalWrite(LEFT_REAR_MOTOR_DIR, HIGH);
      analogWrite(LEFT_REAR_PWM, abs(pwm));
    }
  }
  else if (motor == RIGHT_REAR) {
    if (pwm >= 0) {
      digitalWrite(RIGHT_REAR_MOTOR_DIR, HIGH);
      analogWrite(RIGHT_REAR_PWM, pwm);
    } else {
      digitalWrite(RIGHT_REAR_MOTOR_DIR, LOW);
      analogWrite(RIGHT_REAR_PWM, abs(pwm));
    }
  }
}

// Convert linear and angular velocity to expected encoder ticks
void updateExpectedTicks() {
  float v = goal_velocity[0];
  float w = goal_velocity[1];
  
  // Calculate expected wheel velocities in m/s
  float left_vel = v - (w * WHEEL_SEPARATION / 2.0);
  float right_vel = v + (w * WHEEL_SEPARATION / 2.0);
  
  // Convert velocities to expected ticks per interval (dt = 0.02s)
  float dt = 0.02;  // 20ms timer interval
  float distance_per_tick = (2.0 * PI * WHEEL_RADIUS) / ENCODER_COUNTS_PER_REV;
  
  expected_ticks_left = (int32_t)(left_vel * dt / distance_per_tick);
  expected_ticks_right = (int32_t)(right_vel * dt / distance_per_tick);
}

void updatePID() {
  // Update expected ticks based on current commanded velocity
  updateExpectedTicks();
  
  // Calculate PID for left front motor
  float error_lf = expected_ticks_left - wheels_spin_msg.data.data[LEFT_FRONT];
  integral[LEFT_FRONT] += error_lf;
  float derivative_lf = error_lf - prev_error[LEFT_FRONT];
  pwm_offsets[LEFT_FRONT] = (KP * error_lf) + (KI * integral[LEFT_FRONT]) + (KD * derivative_lf);
  prev_error[LEFT_FRONT] = error_lf;
  
  // Calculate PID for left rear motor
  float error_lr = expected_ticks_left - wheels_spin_msg.data.data[LEFT_REAR];
  integral[LEFT_REAR] += error_lr;
  float derivative_lr = error_lr - prev_error[LEFT_REAR];
  pwm_offsets[LEFT_REAR] = (KP * error_lr) + (KI * integral[LEFT_REAR]) + (KD * derivative_lr);
  prev_error[LEFT_REAR] = error_lr;
  
  // Calculate PID for right front motor
  float error_rf = expected_ticks_right - wheels_spin_msg.data.data[RIGHT_FRONT];
  integral[RIGHT_FRONT] += error_rf;
  float derivative_rf = error_rf - prev_error[RIGHT_FRONT];
  pwm_offsets[RIGHT_FRONT] = (KP * error_rf) + (KI * integral[RIGHT_FRONT]) + (KD * derivative_rf);
  prev_error[RIGHT_FRONT] = error_rf;
  
  // Calculate PID for right rear motor
  float error_rr = expected_ticks_right - wheels_spin_msg.data.data[RIGHT_REAR];
  integral[RIGHT_REAR] += error_rr;
  float derivative_rr = error_rr - prev_error[RIGHT_REAR];
  pwm_offsets[RIGHT_REAR] = (KP * error_rr) + (KI * integral[RIGHT_REAR]) + (KD * derivative_rr);
  prev_error[RIGHT_REAR] = error_rr;
  
  // Limit integral windup for all motors
  for(int i = 0; i < 4; i++) {
    if(integral[i] > 255) integral[i] = 255;
    if(integral[i] < -255) integral[i] = -255;
  }
  
  // Debug message for PID values
  char pid_buffer[100];
  snprintf(pid_buffer, sizeof(pid_buffer), "Target L:%ld R:%ld Err LF:%0.1f LR:%0.1f RF:%0.1f RR:%0.1f", 
           expected_ticks_left, expected_ticks_right,
           error_lf, error_lr, error_rf, error_rr);
  rosidl_runtime_c__String__assign(&chatter_msg.data, pid_buffer);
  rcl_publish(&chatter_publisher, &chatter_msg, NULL);
}

// ------------------------------
// Differential Drive: Compute and apply motor commands from cmd_vel
// ------------------------------
void applyCmdVel() {
  float v = goal_velocity[0];
  float w = goal_velocity[1];
  float left_speed = v - (w * WHEEL_SEPARATION / 2.0);
  float right_speed = v + (w * WHEEL_SEPARATION / 2.0);
  
  int base_left_pwm = (int) constrain((left_speed / MAX_LINEAR_VELOCITY) * 255, -255, 255);
  int base_right_pwm = (int) constrain((right_speed / MAX_LINEAR_VELOCITY) * 255, -255, 255);
  
  // Apply PID corrections to each motor
  int left_front_pwm = constrain(base_left_pwm + pwm_offsets[LEFT_FRONT], -255, 255);
  int left_rear_pwm = constrain(base_left_pwm + pwm_offsets[LEFT_REAR], -255, 255);
  int right_front_pwm = constrain(base_right_pwm + pwm_offsets[RIGHT_FRONT], -255, 255);
  int right_rear_pwm = constrain(base_right_pwm + pwm_offsets[RIGHT_REAR], -255, 255);
  
  MOTOR_PWM_CONTROL(LEFT_FRONT, left_front_pwm);
  MOTOR_PWM_CONTROL(LEFT_REAR, left_rear_pwm);
  MOTOR_PWM_CONTROL(RIGHT_FRONT, right_front_pwm);
  MOTOR_PWM_CONTROL(RIGHT_REAR, right_rear_pwm);
  
  // Update debug message
  char pwm_buffer[100];
  snprintf(pwm_buffer, sizeof(pwm_buffer), "LF: %d, LR: %d, RF: %d, RR: %d", 
           left_front_pwm, left_rear_pwm, right_front_pwm, right_rear_pwm);
  rosidl_runtime_c__String__assign(&chatter_msg.data, pwm_buffer);
  rcl_publish(&chatter_publisher, &chatter_msg, NULL);
}


// ------------------------------
// Timer Callback: Publish odometry, TF, and wheels_spin messages
// ------------------------------
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;
  
  float dt = 0.02;  // 20ms

  int32_t delta_rf = encoder_data[RIGHT_FRONT] - last_encoder_data[RIGHT_FRONT];
  int32_t delta_lf = encoder_data[LEFT_FRONT] - last_encoder_data[LEFT_FRONT];
  int32_t delta_lr = encoder_data[LEFT_REAR] - last_encoder_data[LEFT_REAR];
  int32_t delta_rr = encoder_data[RIGHT_REAR] - last_encoder_data[RIGHT_REAR];

  last_encoder_data[RIGHT_FRONT] = encoder_data[RIGHT_FRONT];
  last_encoder_data[LEFT_FRONT] = encoder_data[LEFT_FRONT];
  last_encoder_data[LEFT_REAR] = encoder_data[LEFT_REAR];
  last_encoder_data[RIGHT_REAR] = encoder_data[RIGHT_REAR];

  int32_t delta_left = (delta_lf + delta_lr) / 2;
  int32_t delta_right = (delta_rf + delta_rr) / 2;

  float distance_per_tick = (2.0 * PI * WHEEL_RADIUS) / ENCODER_COUNTS_PER_REV;
  float left_distance = delta_left * distance_per_tick;
  float right_distance = delta_right * distance_per_tick;

  float delta_s = (left_distance + right_distance) / 2.0;
  float delta_theta = (right_distance - left_distance) / WHEEL_SEPARATION;

  x_pos += delta_s * cos(theta + delta_theta / 2.0);
  y_pos += delta_s * sin(theta + delta_theta / 2.0);
  theta += delta_theta;

  unsigned long ms = millis();
  // Set frame IDs to avoid empty frame errors
  odom_msg.header.frame_id.data = (char*)"odom";
  odom_msg.child_frame_id.data = (char*)"base_link";
  transform_msg.header.frame_id.data = (char*)"odom";
  transform_msg.child_frame_id.data = (char*)"base_link";

  odom_msg.header.stamp.sec = ms / 1000;
  odom_msg.header.stamp.nanosec = (ms % 1000) * 1000000;

  odom_msg.pose.pose.position.x = x_pos;
  odom_msg.pose.pose.position.y = y_pos;
  odom_msg.pose.pose.position.z = 0.0;

  float half_theta = theta / 2.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(half_theta);
  odom_msg.pose.pose.orientation.w = cos(half_theta);

  odom_msg.twist.twist.linear.x = delta_s / dt;
  odom_msg.twist.twist.angular.z = delta_theta / dt;

  transform_msg.header.stamp = odom_msg.header.stamp;
  transform_msg.transform.translation.x = x_pos;
  transform_msg.transform.translation.y = y_pos;
  transform_msg.transform.translation.z = 0.0;
  transform_msg.transform.rotation = odom_msg.pose.pose.orientation;

  tf_msg.transforms.data = &transform_msg;
  tf_msg.transforms.size = 1;

  rcl_publish(&odom_publisher, &odom_msg, NULL);
  rcl_publish(&tf_publisher, &tf_msg, NULL);

  // NEW: Publish individual wheel spin (tick differences) on "wheels_spin"
  // For a primitive sequence, access the underlying data pointer via wheels_spin_msg.data.data
  wheels_spin_msg.data.size = 4; // set the size to 4 elements
  wheels_spin_msg.data.data[LEFT_FRONT] = delta_lf;
  wheels_spin_msg.data.data[RIGHT_FRONT] = delta_rf;
  wheels_spin_msg.data.data[LEFT_REAR] = delta_lr;
  wheels_spin_msg.data.data[RIGHT_REAR] = delta_rr;
  rcl_publish(&wheels_spin_publisher, &wheels_spin_msg, NULL);
  updatePID();
}

// ------------------------------
// Command Velocity Callback: Update goal_velocity from cmd_vel message
// ------------------------------
void commandVelocityCallback(const geometry_msgs__msg__Twist * msg) {
  float prev_linear = goal_velocity[0];
  float prev_angular = goal_velocity[1];
  
  goal_velocity[0] = msg->linear.x;
  goal_velocity[1] = msg->angular.z;
  goal_velocity[0] = constrain(goal_velocity[0], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity[1] = constrain(goal_velocity[1], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  
  // Reset PID integrals when velocity command changes significantly
  if (fabs(prev_linear - goal_velocity[0]) > 0.05 || fabs(prev_angular - goal_velocity[1]) > 0.05) {
    for(int i = 0; i < 4; i++) {
      integral[i] = 0.0;
    }
  }
}

// micro-ROS cmd_vel subscription callback
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;
  commandVelocityCallback(msg);
}

// ------------------------------
// micro-ROS Entity Creation: Publishers, Subscription, Timer, Executor
// ------------------------------
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_cmdvel_motor_control", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  RCCHECK(rclc_publisher_init_default(
    &tf_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "tf"));

  RCCHECK(rclc_publisher_init_default(
    &chatter_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "chatter"));

  RCCHECK(rclc_publisher_init_default(
    &wheels_spin_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray),
    "wheels_spin"));

  // Initialize wheels_spin_msg.data as a sequence with capacity 4
  if (!rosidl_runtime_c__int64__Sequence__init(&wheels_spin_msg.data, 4)) {
    return false;
  }
  wheels_spin_msg.data.size = 4;

  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  const unsigned int timer_timeout = 20;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, cmd_vel_callback, ON_NEW_DATA));

  rosidl_runtime_c__String__init(&chatter_msg.data);

  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  
  rcl_publisher_fini(&odom_publisher, &node);
  rcl_publisher_fini(&tf_publisher, &node);
  rcl_publisher_fini(&chatter_publisher, &node);
  rcl_publisher_fini(&wheels_spin_publisher, &node);
  rcl_subscription_fini(&cmd_vel_subscriber, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  
  rosidl_runtime_c__int64__Sequence__fini(&wheels_spin_msg.data);
}

// ------------------------------
// Hardware Initialization: Set motor PWM resolution, pin modes, and attach encoder interrupts
// ------------------------------
void initHardware() {
  analogWriteResolution(PWM_RESOLUTION);
  analogWriteFrequency(LEFT_FRONT_PWM, 375000);
  analogWriteFrequency(RIGHT_FRONT_PWM, 375000);
  analogWriteFrequency(LEFT_REAR_PWM, 375000);
  analogWriteFrequency(RIGHT_REAR_PWM, 375000);

  pinMode(RIGHT_FRONT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_FRONT_ENC_B, INPUT_PULLUP);
  pinMode(LEFT_FRONT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_FRONT_ENC_B, INPUT_PULLUP);
  pinMode(LEFT_REAR_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_REAR_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_REAR_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_REAR_ENC_B, INPUT_PULLUP);

  pinMode(LEFT_FRONT_MOTOR_DIR, OUTPUT);
  pinMode(LEFT_FRONT_PWM, OUTPUT);
  pinMode(RIGHT_FRONT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_FRONT_PWM, OUTPUT);
  pinMode(LEFT_REAR_MOTOR_DIR, OUTPUT);
  pinMode(LEFT_REAR_PWM, OUTPUT);
  pinMode(RIGHT_REAR_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_REAR_PWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_FRONT_ENC_A), MotorISR_LEFT_FRONT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_FRONT_ENC_A), MotorISR_RIGHT_FRONT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_REAR_ENC_A), MotorISR_LEFT_REAR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_REAR_ENC_A), MotorISR_RIGHT_REAR, CHANGE);
}

// ------------------------------
// Setup: Initialize Serial, hardware, and micro-ROS transport
// ------------------------------
void setup() {
  Serial.begin(2000000);
  set_microros_serial_transports(Serial);
  initHardware();
  state = WAITING_AGENT;
}

// ------------------------------
// Main Loop: Process micro-ROS executor and update motor commands based on cmd_vel
// ------------------------------
void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, 
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, 
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        // Update motor commands based on latest goal velocities and publish PWM values.
        applyCmdVel();
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
  digitalWrite(LED_BUILTIN, state == AGENT_CONNECTED);
}
