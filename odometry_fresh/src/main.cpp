#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include "robot_config.h"  // Contains simplified odometry routines

// Define LED and helper macros
#define LED_PIN LED_BUILTIN
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) { return false; } }
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis(); } \
  if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)

// Connection state management
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

// Global micro-ROS variables
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t odom_publisher;
rcl_publisher_t tf_publisher;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__TransformStamped tf_msg;

// To calculate elapsed time for odometry
unsigned long prev_update_time = 0;

// Timer callback: update odometry using sensor routines and publish odom and TF messages
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    unsigned long current_time = millis();
    double dt = (current_time - prev_update_time) * 0.001; // dt in seconds
    if (dt <= 0.0) {
      dt = 0.001; // prevent division by zero
    }
    
    // Calculate new odometry based on sensor data (or simulated values)
    if (calcOdometry(dt)) {
      updateOdometry(); // In this simple version, calcOdometry() already updates the globals.
    }
    
    // Update the TF message based on the new pose
    updateTF(tf_msg);
    
    // Fill the Odometry message header with the current time
    odom_msg.header.stamp.sec = current_time / 1000;
    odom_msg.header.stamp.nanosec = (current_time % 1000) * 1000000;
    odom_msg.header.frame_id.data = "odom";
    odom_msg.child_frame_id.data = "base_link";
    
    // Fill in pose using global odom_pose (x, y, theta)
    odom_msg.pose.pose.position.x = odom_pose[0];
    odom_msg.pose.pose.position.y = odom_pose[1];
    odom_msg.pose.pose.position.z = 0.0; // Assuming flat ground
    
    // Convert yaw (theta) to quaternion (roll and pitch assumed zero)
    float yaw = odom_pose[2];
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(yaw / 2.0);
    odom_msg.pose.pose.orientation.w = cos(yaw / 2.0);
    
    // Fill in twist (velocity) message
    odom_msg.twist.twist.linear.x = odom_vel[0];
    odom_msg.twist.twist.linear.y = odom_vel[1];
    odom_msg.twist.twist.angular.z = odom_vel[2];
    
    // Update the TF message header and translation (assuming updateTF() filled in rotation)
    tf_msg.header.stamp = odom_msg.header.stamp;
    tf_msg.header.frame_id.data = "odom";
    tf_msg.child_frame_id.data = "base_link";
    
    // Publish the odometry and TF messages
    rcl_publish(&odom_publisher, &odom_msg, NULL);
    rcl_publish(&tf_publisher, &tf_msg, NULL);
    
    // Update time for next dt calculation
    prev_update_time = current_time;
  }
}

// Create micro-ROS entities and initialize sensor routines
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_odom_publisher", "", &support));
  
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));
  
  RCCHECK(rclc_publisher_init_default(
    &tf_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TransformStamped),
    "tf"));
  
  // Create a timer with 100ms timeout (10Hz update rate)
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  // Initialize odometry and (if needed) motor-related states
  initOdom();
  initMotorState();
  initMotorVel();
  
  prev_update_time = millis();
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
  set_microros_serial_transports(Serial);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  static states state = WAITING_AGENT;
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
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
  digitalWrite(LED_PIN, (state == AGENT_CONNECTED) ? HIGH : LOW);
}
