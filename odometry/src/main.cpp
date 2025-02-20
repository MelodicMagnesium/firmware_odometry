#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>

// Hardware Definitions
#define RIGHT_FRONT_ENC_A 7
#define RIGHT_FRONT_ENC_B 6
#define LEFT_FRONT_ENC_A 8
#define LEFT_FRONT_ENC_B 9
#define LEFT_REAR_ENC_A 32
#define LEFT_REAR_ENC_B 33
#define RIGHT_REAR_ENC_A 37
#define RIGHT_REAR_ENC_B 36

#define RIGHT_FRONT 0
#define LEFT_FRONT  1
#define LEFT_REAR   2
#define RIGHT_REAR  3

// Robot parameters
#define WHEEL_RADIUS 0.12                // meters
#define WHEEL_SEPARATION 0.122           // meters
#define ENCODER_COUNTS_PER_REV 1700      // counts per revolution

#define LED_PIN LED_BUILTIN
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// Global variables for micro-ROS
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

// State management
enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

// Encoder and odometry variables
volatile int32_t encoder_data[4] = {0, 0, 0, 0};
int32_t last_encoder_data[4] = {0, 0, 0, 0};
float x_pos = 0.0;
float y_pos = 0.0;
float theta = 0.0;

// Encoder ISR Functions
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

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) return;

    // Calculate time delta (assume 50Hz timer = 20ms)
    float dt = 0.02;

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

    // Convert tick differences to distance traveled
    float distance_per_tick = (2.0 * PI * WHEEL_RADIUS) / ENCODER_COUNTS_PER_REV;
    float left_distance  = delta_left  * distance_per_tick;
    float right_distance = delta_right * distance_per_tick;

    // Compute robot displacement and change in orientation
    float delta_s     = (left_distance + right_distance) / 2.0;
    float delta_theta = (right_distance - left_distance) / WHEEL_SEPARATION;

    // Update pose using midpoint integration
    x_pos   += delta_s * cos(theta + delta_theta / 2.0);
    y_pos   += delta_s * sin(theta + delta_theta / 2.0);
    theta   += delta_theta;

    // Get current timestamp using millis() (Teensy doesn't support clock_gettime)
    unsigned long ms = millis();
    odom_msg.header.stamp.sec = ms / 1000;
    odom_msg.header.stamp.nanosec = (ms % 1000) * 1000000;
    
    // Fill pose data
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    
    // Convert theta to quaternion (assuming roll and pitch are zero)
    float half_theta = theta / 2.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(half_theta);
    odom_msg.pose.pose.orientation.w = cos(half_theta);

    // Fill velocity data
    odom_msg.twist.twist.linear.x = delta_s / dt;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = delta_theta / dt;

    // Fill transform message
    transform_msg.header.stamp = odom_msg.header.stamp;
    transform_msg.transform.translation.x = x_pos;
    transform_msg.transform.translation.y = y_pos;
    transform_msg.transform.translation.z = 0.0;
    transform_msg.transform.rotation = odom_msg.pose.pose.orientation;

    // Update TF message
    tf_msg.transforms.data = &transform_msg;
    tf_msg.transforms.size = 1;

    // Publish messages
    rcl_publish(&odom_publisher, &odom_msg, NULL);
    rcl_publish(&tf_publisher, &tf_msg, NULL);
}


bool create_entities() {
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_encoder_odom", "", &support));

    // Create publishers
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

    // Create timer (50Hz)
    const unsigned int timer_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // Create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    return true;
}

void destroy_entities() {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    
    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&tf_publisher, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

void setup() {
    // Initialize serial and LED
    set_microros_serial_transports(Serial);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Initialize encoder pins
    pinMode(RIGHT_FRONT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_FRONT_ENC_B, INPUT_PULLUP);
    pinMode(LEFT_FRONT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_FRONT_ENC_B, INPUT_PULLUP);
    pinMode(LEFT_REAR_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_REAR_ENC_B, INPUT_PULLUP);
    pinMode(RIGHT_REAR_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_REAR_ENC_B, INPUT_PULLUP);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(RIGHT_FRONT_ENC_A), MotorISR_RIGHT_FRONT, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_FRONT_ENC_A),  MotorISR_LEFT_FRONT,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_REAR_ENC_A),   MotorISR_LEFT_REAR,   CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_REAR_ENC_A),  MotorISR_RIGHT_REAR,  CHANGE);
    
    state = WAITING_AGENT;

    // Initialize message strings
    odom_msg.header.frame_id.data = (char*)"odom";
    odom_msg.header.frame_id.capacity = strlen("odom");
    odom_msg.child_frame_id.data = (char*)"base_link";
    odom_msg.child_frame_id.capacity = strlen("base_link");
    
    transform_msg.header.frame_id.data = (char*)"odom";
    transform_msg.header.frame_id.capacity = strlen("odom");
    transform_msg.child_frame_id.data = (char*)"base_link";
    transform_msg.child_frame_id.capacity = strlen("base_link");

    // Initialize TF message
    tf_msg.transforms.capacity = 1;
}

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
            }
            break;
        case AGENT_DISCONNECTED:
            destroy_entities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }

    digitalWrite(LED_PIN, state == AGENT_CONNECTED);
}