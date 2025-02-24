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
#include <rosidl_runtime_c/string_functions.h>

// ------------------------------
// Robot & Motor Configuration Constants
// ------------------------------

// Motor indices
#define RIGHT_FRONT 0
#define LEFT_FRONT  1
#define LEFT_REAR   2
#define RIGHT_REAR  3

// Motor Pin Definitions (as in robot_config.h)
#define RIGHT_FRONT_ENC_A 6
#define RIGHT_FRONT_ENC_B 7
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

#define RIGHT_REAR_ENC_A 36
#define RIGHT_REAR_ENC_B 37
#define RIGHT_REAR_MOTOR_DIR 41
#define RIGHT_REAR_PWM 5

// PWM and resolution
#define PWM_RESOLUTION 8    // 8-bit PWM (0-255)
  
// Robot physical parameters (from robot_config.h)
#define WHEEL_RADIUS 0.33   // meters
#define WHEEL_SEPARATION 0.122  // meters
#define TURNING_RADIUS 0.080    // meters
#define ENCODER_COUNTS_PER_REV 1700
#define MAX_RPM 300

// Velocity limits (computed from above)
#define MAX_LINEAR_VELOCITY (WHEEL_RADIUS * 2 * PI * MAX_RPM / 60)
#define MAX_ANGULAR_VELOCITY (MAX_LINEAR_VELOCITY / TURNING_RADIUS)
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

// Target velocities updated via cmd_vel [LINEAR_X, ANGULAR]
float goal_velocity[2] = {0.0, 0.0};

// ------------------------------
// Micro-ROS globals
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

// (Optional) For debugging: publisher on "chatter" (we use this to republish cmd_vel as a string)
rcl_publisher_t chatter_publisher;
std_msgs__msg__String chatter_msg;

// ------------------------------
// Connection state
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
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
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
  // Limit pwm to 8-bit range (-255 to 255)
  if (pwm > 255) {
    pwm = 255;
  } else if (pwm < -255) {
    pwm = -255;
  }
  
  // Depending on motor index, set direction and PWM value
  if (motor == LEFT_FRONT) {
    if (pwm >= 0) {
      digitalWrite(LEFT_FRONT_MOTOR_DIR, HIGH);
      analogWrite(LEFT_FRONT_PWM, pwm);
    } else {
      digitalWrite(LEFT_FRONT_MOTOR_DIR, LOW);
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
      digitalWrite(LEFT_REAR_MOTOR_DIR, HIGH);
      analogWrite(LEFT_REAR_PWM, pwm);
    } else {
      digitalWrite(LEFT_REAR_MOTOR_DIR, LOW);
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

// ------------------------------
// Differential Drive: Compute and apply motor commands from cmd_vel
// ------------------------------
void applyCmdVel() {
  // Differential drive kinematics:
  // v_left = v - (w * wheel_separation / 2)
  // v_right = v + (w * wheel_separation / 2)
  float v = goal_velocity[0];
  float w = goal_velocity[1];
  float left_speed = v - (w * WHEEL_SEPARATION / 2.0);
  float right_speed = v + (w * WHEEL_SEPARATION / 2.0);
  
  // Map speeds to PWM values (assume MAX_LINEAR_VELOCITY corresponds to full scale, 255)
  int left_pwm = (int) constrain((left_speed / MAX_LINEAR_VELOCITY) * 255, -255, 255);
  int right_pwm = (int) constrain((right_speed / MAX_LINEAR_VELOCITY) * 255, -255, 255);
  
  MOTOR_PWM_CONTROL(LEFT_FRONT, left_pwm);
  MOTOR_PWM_CONTROL(LEFT_REAR, left_pwm);
  MOTOR_PWM_CONTROL(RIGHT_FRONT, right_pwm);
  MOTOR_PWM_CONTROL(RIGHT_REAR, right_pwm);
}

// ------------------------------
// Micro-ROS Timer Callback: Publish odom and TF messages
// ------------------------------
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  float dt = 0.02;  // Timer period: 20ms

  // Compute tick differences for each wheel
  int32_t delta_rf = encoder_data[RIGHT_FRONT] - last_encoder_data[RIGHT_FRONT];
  int32_t delta_lf = encoder_data[LEFT_FRONT]  - last_encoder_data[LEFT_FRONT];
  int32_t delta_lr = encoder_data[LEFT_REAR]   - last_encoder_data[LEFT_REAR];
  int32_t delta_rr = encoder_data[RIGHT_REAR]  - last_encoder_data[RIGHT_REAR];

  // Update last encoder counts
  last_encoder_data[RIGHT_FRONT] = encoder_data[RIGHT_FRONT];
  last_encoder_data[LEFT_FRONT]  = encoder_data[LEFT_FRONT];
  last_encoder_data[LEFT_REAR]   = encoder_data[LEFT_REAR];
  last_encoder_data[RIGHT_REAR]  = encoder_data[RIGHT_REAR];

  // Average ticks for left and right sides
  int32_t delta_left  = (delta_lf + delta_lr) / 2;
  int32_t delta_right = (delta_rf + delta_rr) / 2;

  // Convert ticks to distance
  float distance_per_tick = (2.0 * PI * WHEEL_RADIUS) / ENCODER_COUNTS_PER_REV;
  float left_distance = delta_left * distance_per_tick;
  float right_distance = delta_right * distance_per_tick;

  float delta_s = (left_distance + right_distance) / 2.0;
  float delta_theta = (right_distance - left_distance) / WHEEL_SEPARATION;

  // Update odometry (using midpoint integration)
  x_pos += delta_s * cos(theta + delta_theta / 2.0);
  y_pos += delta_s * sin(theta + delta_theta / 2.0);
  theta += delta_theta;

  unsigned long ms = millis();
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
}

// ------------------------------
// Command Velocity Callback (updates global goal_velocity)
// ------------------------------
void commandVelocityCallback(const geometry_msgs__msg__Twist * msg) {
  goal_velocity[0] = msg->linear.x;
  goal_velocity[1] = msg->angular.z;

  // Constrain values
  goal_velocity[0] = constrain(goal_velocity[0], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity[1] = constrain(goal_velocity[1], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
}

// micro-ROS cmd_vel subscription callback
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;
  commandVelocityCallback(msg);
  
  // (Optional) Republish a debug string on "chatter"
  char buffer[100];
  snprintf(buffer, sizeof(buffer), "cmd_vel: linear.x=%.2f, angular.z=%.2f", msg->linear.x, msg->angular.z);
  rosidl_runtime_c__String__assign(&chatter_msg.data, buffer);
  rcl_publish(&chatter_publisher, &chatter_msg, NULL);
}

// ------------------------------
// micro-ROS entity creation: publishers, subscription, timer, executor
// ------------------------------
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_cmdvel_motor_control", "", &support));

  // Create publishers for odom, tf, and chatter
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

  // Create subscription for cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Create timer (50Hz)
  const unsigned int timer_timeout = 20;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create executor (2 handles: timer and subscription)
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, cmd_vel_callback, ON_NEW_DATA));

  // Initialize chatter message string (pre-allocate an empty string)
  rosidl_runtime_c__String__init(&chatter_msg.data);

  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  
  rcl_publisher_fini(&odom_publisher, &node);
  rcl_publisher_fini(&tf_publisher, &node);
  rcl_publisher_fini(&chatter_publisher, &node);
  rcl_subscription_fini(&cmd_vel_subscriber, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ------------------------------
// Hardware Initialization: motor pins, encoder pins, PWM setup, etc.
// ------------------------------
void initHardware() {
  // Set motor PWM resolution and frequencies
  analogWriteResolution(PWM_RESOLUTION);
  analogWriteFrequency(LEFT_FRONT_PWM, 375000);
  analogWriteFrequency(RIGHT_FRONT_PWM, 375000);
  analogWriteFrequency(LEFT_REAR_PWM, 375000);
  analogWriteFrequency(RIGHT_REAR_PWM, 375000);

  // Set encoder pins (INPUT_PULLUP)
  pinMode(RIGHT_FRONT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_FRONT_ENC_B, INPUT_PULLUP);
  pinMode(LEFT_FRONT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_FRONT_ENC_B, INPUT_PULLUP);
  pinMode(LEFT_REAR_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_REAR_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_REAR_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_REAR_ENC_B, INPUT_PULLUP);

  // Set motor direction and PWM pins as OUTPUT
  pinMode(LEFT_FRONT_MOTOR_DIR, OUTPUT);
  pinMode(LEFT_FRONT_PWM, OUTPUT);
  
  pinMode(RIGHT_FRONT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_FRONT_PWM, OUTPUT);
  
  pinMode(LEFT_REAR_MOTOR_DIR, OUTPUT);
  pinMode(LEFT_REAR_PWM, OUTPUT);
  
  pinMode(RIGHT_REAR_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_REAR_PWM, OUTPUT);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_FRONT_ENC_A), MotorISR_LEFT_FRONT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_FRONT_ENC_A), MotorISR_RIGHT_FRONT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_REAR_ENC_A), MotorISR_LEFT_REAR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_REAR_ENC_A), MotorISR_RIGHT_REAR, CHANGE);
}

// ------------------------------
// Setup: initialize hardware and micro-ROS
// ------------------------------
void setup() {
  // Initialize Serial and micro-ROS transport
  Serial.begin(2000000);
  set_microros_serial_transports(Serial);

  // Initialize hardware (motors, encoders, PWM)
  initHardware();

  state = WAITING_AGENT;
}

// ------------------------------
// Main Loop: Process micro-ROS and update motor commands based on cmd_vel
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
        // Apply the latest command velocities to drive motors
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
