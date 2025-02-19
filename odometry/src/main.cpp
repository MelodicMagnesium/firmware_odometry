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

#define LED_PIN LED_BUILTIN
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// Global variables
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t odom_publisher;
rcl_publisher_t tf_publisher;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__TransformStamped tf_msg;

// State management
enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

// Global position tracking
static float x = 0.0;
static float y = 0.0;
static float theta = 0.0;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Update robot position (simulated motion)
        x += 0.01;
        y += 0.005;
        theta += 0.01;

        // Fill odometry message
        odom_msg.header.stamp.sec = millis()/1000;
        odom_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
        
        // Using simplified quaternion for 2D rotation
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = sin(theta/2);
        odom_msg.pose.pose.orientation.w = cos(theta/2);
        
        odom_msg.twist.twist.linear.x = 0.1;
        odom_msg.twist.twist.angular.z = 0.1;

        // Fill TF message
        tf_msg.header.stamp = odom_msg.header.stamp;
        tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
        tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
        tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

        // Publish messages
        rcl_publish(&odom_publisher, &odom_msg, NULL);
        rcl_publish(&tf_publisher, &tf_msg, NULL);
    }
}

bool create_entities() {
    allocator = rcl_get_default_allocator();

    // Create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    RCCHECK(rclc_node_init_default(&node, "teensy_odom_publisher", "", &support));

    // Create publishers
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

    // Create timer
    const unsigned int timer_timeout = 100;
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
    set_microros_serial_transports(Serial);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    state = WAITING_AGENT;

    // Initialize message strings
    odom_msg.header.frame_id.data = (char*)"odom";
    odom_msg.header.frame_id.capacity = strlen("odom");
    odom_msg.child_frame_id.data = (char*)"base_link";
    odom_msg.child_frame_id.capacity = strlen("base_link");
    
    tf_msg.header.frame_id.data = (char*)"odom";
    tf_msg.header.frame_id.capacity = strlen("odom");
    tf_msg.child_frame_id.data = (char*)"base_link";
    tf_msg.child_frame_id.capacity = strlen("base_link");
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

    // Update LED based on connection state
    digitalWrite(LED_PIN, state == AGENT_CONNECTED);
}