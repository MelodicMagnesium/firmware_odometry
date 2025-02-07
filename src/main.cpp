#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int64MultiArray.h>

// --- Encoder Pin Definitions ---
// These are chosen to match your hardware design.
// You may need to adjust them for your wiring.
#define RIGHT_FRONT_ENC_A 6
#define RIGHT_FRONT_ENC_B 7

#define LEFT_FRONT_ENC_A 8
#define LEFT_FRONT_ENC_B 9

#define LEFT_REAR_ENC_A 32
#define LEFT_REAR_ENC_B 33

#define RIGHT_REAR_ENC_A 36
#define RIGHT_REAR_ENC_B 37

// Motor (or wheel) indices
#define RIGHT_FRONT 0
#define LEFT_FRONT  1
#define LEFT_REAR   2
#define RIGHT_REAR  3
#define NUM_WHEELS  4

// Global encoder counts for four wheels.
volatile long encoder_counts[NUM_WHEELS] = {0, 0, 0, 0};

// --- Interrupt Service Routines ---
// Each ISR reads the "B" channel to determine rotation direction.
// A simple quadrature decoding method is used.

void ISR_RightFront() {
  int b = digitalRead(RIGHT_FRONT_ENC_B);
  // If channel A equals channel B, count up; otherwise, count down.
  if (digitalRead(RIGHT_FRONT_ENC_A) == b)
    encoder_counts[RIGHT_FRONT]++;
  else
    encoder_counts[RIGHT_FRONT]--;
}

void ISR_LeftFront() {
  int b = digitalRead(LEFT_FRONT_ENC_B);
  if (digitalRead(LEFT_FRONT_ENC_A) == b)
    encoder_counts[LEFT_FRONT]++;
  else
    encoder_counts[LEFT_FRONT]--;
}

void ISR_LeftRear() {
  int b = digitalRead(LEFT_REAR_ENC_B);
  if (digitalRead(LEFT_REAR_ENC_A) == b)
    encoder_counts[LEFT_REAR]++;
  else
    encoder_counts[LEFT_REAR]--;
}

void ISR_RightRear() {
  int b = digitalRead(RIGHT_REAR_ENC_B);
  if (digitalRead(RIGHT_REAR_ENC_A) == b)
    encoder_counts[RIGHT_REAR]++;
  else
    encoder_counts[RIGHT_REAR]--;
}

// --- ROS Setup ---
ros::NodeHandle nh;
std_msgs::Int64MultiArray encoder_msg;
ros::Publisher encoder_pub("motor_encoder", &encoder_msg);

// Publishing period (in ms)
const unsigned long PUBLISH_PERIOD = 100; // Publish every 100 ms
unsigned long last_publish_time = 0;

void setup() {
  // Initialize Serial for rosserial communication.
  Serial.begin(115200);
  while (!Serial) ;  // Wait for Serial port to be ready (optional)

  // Initialize encoder pins.
  // Use INPUT_PULLUP for A channels (and B channels if needed).
  pinMode(RIGHT_FRONT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_FRONT_ENC_B, INPUT_PULLUP);
  
  pinMode(LEFT_FRONT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_FRONT_ENC_B, INPUT_PULLUP);
  
  pinMode(LEFT_REAR_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_REAR_ENC_B, INPUT_PULLUP);
  
  pinMode(RIGHT_REAR_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_REAR_ENC_B, INPUT_PULLUP);
  
  // Attach interrupts for encoder A channels.
  attachInterrupt(digitalPinToInterrupt(RIGHT_FRONT_ENC_A), ISR_RightFront, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_FRONT_ENC_A), ISR_LeftFront, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_REAR_ENC_A), ISR_LeftRear, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_REAR_ENC_A), ISR_RightRear, CHANGE);

  // Initialize ROS node handle.
  nh.initNode();
  nh.advertise(encoder_pub);

  // Prepare the message array.
  // We use a multi-array with NUM_WHEELS elements.
  encoder_msg.data_length = NUM_WHEELS;
  // Optionally, allocate a data array if required (using the default allocator here)
  static long encoder_data_buffer[NUM_WHEELS] = {0, 0, 0, 0};
  encoder_msg.data = (int64_t *)encoder_data_buffer;
}

void loop() {
  nh.spinOnce();

  // Publish encoder data at defined intervals.
  unsigned long now = millis();
  if (now - last_publish_time >= PUBLISH_PERIOD) {
    last_publish_time = now;
    
    // Copy the volatile encoder counts to a local array to avoid reading during update.
    int64_t counts[NUM_WHEELS];
    noInterrupts();
    for (int i = 0; i < NUM_WHEELS; i++) {
      counts[i] = encoder_counts[i];
    }
    interrupts();
    
    // Update the message data.
    for (int i = 0; i < NUM_WHEELS; i++) {
      // Because we allocated a static array earlier, we can update the values in place.
      ((int64_t *)encoder_msg.data)[i] = counts[i];
    }
    
    // Publish the message.
    encoder_pub.publish(&encoder_msg);
  }
}
