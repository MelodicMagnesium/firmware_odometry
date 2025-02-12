#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>  // For string assignment

// Uncomment if needed
// #define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do {            \
  static volatile int64_t init = -1;                 \
  if (init == -1) { init = uxr_millis(); }           \
  if (uxr_millis() - init > MS) { X; init = uxr_millis(); }  \
} while (0)

// ---------------- Micro-ROS Variables ----------------
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
std_msgs__msg__String msg;
bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// ---------------- Encoder & Kinematics Definitions ----------------
#define WHEEL_NUM 4
#define RIGHT_FRONT 0
#define LEFT_FRONT  1
#define LEFT_REAR   2
#define RIGHT_REAR  3

#define ENCODER_COUNTS_PER_REV 1700  // counts per wheel revolution
#define UPDATE_INTERVAL 500          // ms over which velocity is computed

// Global encoder tick counters and previous values
volatile long encoder_data[WHEEL_NUM] = {0, 0, 0, 0};
long previous_encoder_data[WHEEL_NUM] = {0, 0, 0, 0};

// ---------------- Encoder ISR Functions ----------------
// These functions update the tick counts using digitalReadFast.
// Adjust the pin numbers as needed for your hardware.
void encoderISR_RightFront() {
  int b = digitalReadFast(7); // RIGHT_FRONT_ENC_B (pin 7)
  if (digitalReadFast(6) == b)
    encoder_data[RIGHT_FRONT]++;
  else
    encoder_data[RIGHT_FRONT]--;
}

void encoderISR_LeftFront() {
  int b = digitalReadFast(9); // LEFT_FRONT_ENC_B (pin 9)
  if (digitalReadFast(8) == b)
    encoder_data[LEFT_FRONT]++;
  else
    encoder_data[LEFT_FRONT]--;
}

void encoderISR_LeftRear() {
  int b = digitalReadFast(33); // LEFT_REAR_ENC_B (pin 33)
  if (digitalReadFast(32) == b)
    encoder_data[LEFT_REAR]++;
  else
    encoder_data[LEFT_REAR]--;
}

void encoderISR_RightRear() {
  int b = digitalReadFast(37); // RIGHT_REAR_ENC_B (pin 37)
  if (digitalReadFast(36) == b)
    encoder_data[RIGHT_REAR]++;
  else
    encoder_data[RIGHT_REAR]--;
}

// ---------------- Timer Callback ----------------
// Every 1 second this callback computes for each wheel:
// - Total rotations (encoder_data / ENCODER_COUNTS_PER_REV)
// - Estimated velocity (rotations per second)
// It then builds a text string summarizing the data and publishes it on "chatter".
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    String outStr = "";
    for (int i = 0; i < WHEEL_NUM; i++) {
      long current = encoder_data[i];
      long delta = current - previous_encoder_data[i];
      previous_encoder_data[i] = current;
      float rotations = (float) current / ENCODER_COUNTS_PER_REV;
      float velocity_rps = ((float) delta / ENCODER_COUNTS_PER_REV) * (1000.0 / UPDATE_INTERVAL);
      
      // Label each wheel
      switch(i) {
        case RIGHT_FRONT:
          outStr += "RF: Rot=";
          break;
        case LEFT_FRONT:
          outStr += "LF: Rot=";
          break;
        case LEFT_REAR:
          outStr += "LR: Rot=";
          break;
        case RIGHT_REAR:
          outStr += "RR: Rot=";
          break;
      }
      outStr += String(rotations, 4);
      outStr += ", Vel=";
      outStr += String(velocity_rps, 4);
      if (i < WHEEL_NUM - 1) {
        outStr += " || ";
      }
    }
    
    // Assign the built string to the message using the helper function.
    rosidl_runtime_c__String__assign(&msg.data, outStr.c_str());
    // Publish the message.
    rcl_publish(&publisher, &msg, NULL);
  }
}

// ---------------- Micro-ROS Entities Creation ----------------
bool create_entities()
{
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "encoder_text_publisher", "", &support));
  
  // Create publisher on "chatter" using std_msgs/String
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "chatter"));
  
  // Create a timer with a 1000 ms period that calls timer_callback
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  
  rcl_publisher_fini(&publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ---------------- Setup Function ----------------
void setup() {
  // Initialize micro-ROS serial transports.
  set_microros_serial_transports(Serial);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Configure encoder pins as inputs with internal pull-ups.
  // Encoder A channels:
  pinMode(6, INPUT_PULLUP);   // RIGHT_FRONT_ENC_A
  pinMode(8, INPUT_PULLUP);   // LEFT_FRONT_ENC_A
  pinMode(32, INPUT_PULLUP);  // LEFT_REAR_ENC_A
  pinMode(36, INPUT_PULLUP);  // RIGHT_REAR_ENC_A
  // Encoder B channels:
  pinMode(7, INPUT_PULLUP);   // RIGHT_FRONT_ENC_B
  pinMode(9, INPUT_PULLUP);   // LEFT_FRONT_ENC_B
  pinMode(33, INPUT_PULLUP);  // LEFT_REAR_ENC_B
  pinMode(37, INPUT_PULLUP);  // RIGHT_REAR_ENC_B
  
  // Attach interrupts on the encoder A channels.
  attachInterrupt(digitalPinToInterrupt(6), encoderISR_RightFront, CHANGE);
  attachInterrupt(digitalPinToInterrupt(8), encoderISR_LeftFront, CHANGE);
  attachInterrupt(digitalPinToInterrupt(32), encoderISR_LeftRear, CHANGE);
  attachInterrupt(digitalPinToInterrupt(36), encoderISR_RightRear, CHANGE);
  
  state = WAITING_AGENT;
  
  // Initialize the std_msgs/String message.
  if (!rosidl_runtime_c__String__init(&msg.data)) {
    Serial.println("Failed to initialize string!");
  }
  // Optionally assign an initial message.
  rosidl_runtime_c__String__assign(&msg.data, "Initial encoder data");
}

// ---------------- Main Loop ----------------
void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
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
  
  // LED indicator: ON when connected.
  digitalWrite(LED_BUILTIN, (state == AGENT_CONNECTED) ? HIGH : LOW);
}
