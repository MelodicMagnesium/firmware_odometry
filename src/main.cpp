#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
// #include <tf/transform_datatypes.h>

// --- Robot Parameters ---
// (These values are based on your robot_config.h)
#define WHEEL_RADIUS 0.33             // meters
#define ENCODER_COUNTS_PER_REV 1700   // counts per full wheel revolution
#define WHEEL_SEPARATION 0.122        // meters (distance between left and right wheels)
#define TICK2RAD 0.00394177           // radians per encoder tick (from your config)
#define PI 3.14159265359

// --- Encoder Data ---
// We assume you have four encoders, one per wheel. The indices are defined as follows:
#define RIGHT_FRONT 0
#define LEFT_FRONT  1
#define LEFT_REAR   2
#define RIGHT_REAR  3
#define WHEEL_NUM   4

// Global encoder count array (updated by your encoder ISRs)
volatile long encoder_data[WHEEL_NUM] = {0, 0, 0, 0};

// For differential-drive odometry, we average the counts of the two wheels on each side.
long last_left_count = 0;   // Average of LEFT_FRONT and LEFT_REAR
long last_right_count = 0;  // Average of RIGHT_FRONT and RIGHT_REAR

// --- Odometry State ---
double x_pos = 0.0;   // meters
double y_pos = 0.0;   // meters
double theta = 0.0;   // robot heading in radians

// --- ROS Objects ---
ros::NodeHandle nh;
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);
tf::TransformBroadcaster tf_broadcaster;
geometry_msgs::TransformStamped odom_tf;

// Timing
unsigned long last_time = 0;
const double PUBLISH_PERIOD = 0.1; // seconds (10 Hz)

// -----------------------------------------------------------------------------
// Encoder ISR functions
// (These should be attached to the proper encoder A channel pins using attachInterrupt)
// For simplicity, here are dummy ISR functions. In your actual code, these ISRs
// update encoder_data[] for each wheel.
void MotorISR_RIGHT_FRONT() {
  // Example: simple quadrature decoding (replace with your actual logic)
  int b = digitalRead(7);  // Assuming RIGHT_FRONT_ENC_B is on pin 7
  if (digitalRead(6))  // RIGHT_FRONT_ENC_A on pin 6
    b ? encoder_data[RIGHT_FRONT]-- : encoder_data[RIGHT_FRONT]++;
  else
    b ? encoder_data[RIGHT_FRONT]++ : encoder_data[RIGHT_FRONT]--;
}

void MotorISR_LEFT_FRONT() {
  int b = digitalRead(9);  // LEFT_FRONT_ENC_B on pin 9
  if (digitalRead(8))  // LEFT_FRONT_ENC_A on pin 8
    b ? encoder_data[LEFT_FRONT]-- : encoder_data[LEFT_FRONT]++;
  else
    b ? encoder_data[LEFT_FRONT]++ : encoder_data[LEFT_FRONT]--;
}

void MotorISR_LEFT_REAR() {
  int b = digitalRead(33);  // LEFT_REAR_ENC_B on pin 33
  if (digitalRead(32))  // LEFT_REAR_ENC_A on pin 32
    b ? encoder_data[LEFT_REAR]-- : encoder_data[LEFT_REAR]++;
  else
    b ? encoder_data[LEFT_REAR]++ : encoder_data[LEFT_REAR]--;
}

void MotorISR_RIGHT_REAR() {
  int b = digitalRead(37);  // RIGHT_REAR_ENC_B on pin 37
  if (digitalRead(36))  // RIGHT_REAR_ENC_A on pin 36
    b ? encoder_data[RIGHT_REAR]-- : encoder_data[RIGHT_REAR]++;
  else
    b ? encoder_data[RIGHT_REAR]++ : encoder_data[RIGHT_REAR]--;
}

// Helper function to create a quaternion from a yaw angle (in radians)
geometry_msgs::Quaternion createQuaternionFromYaw(double yaw) {
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = sin(yaw / 2.0);
  q.w = cos(yaw / 2.0);
  return q;
}


// -----------------------------------------------------------------------------
// Setup function
void setup() {
  Serial.begin(115200);

  // Initialize ROS node
  nh.initNode();
  nh.advertise(odom_pub);
  tf_broadcaster.init(nh);

  // --- Encoder Pin Initialization ---  
  // (Define the pin numbers here as in your robot_config.h)
  // RIGHT_FRONT_ENC_A on pin 6, RIGHT_FRONT_ENC_B on pin 7, etc.
  pinMode(6, INPUT_PULLUP);   // RIGHT_FRONT_ENC_A
  pinMode(7, INPUT_PULLUP);   // RIGHT_FRONT_ENC_B

  pinMode(8, INPUT_PULLUP);   // LEFT_FRONT_ENC_A
  pinMode(9, INPUT_PULLUP);   // LEFT_FRONT_ENC_B

  pinMode(32, INPUT_PULLUP);  // LEFT_REAR_ENC_A
  pinMode(33, INPUT_PULLUP);  // LEFT_REAR_ENC_B

  pinMode(36, INPUT_PULLUP);  // RIGHT_REAR_ENC_A
  pinMode(37, INPUT_PULLUP);  // RIGHT_REAR_ENC_B

  // Attach interrupts to encoder A channels (assuming these pins are interrupt-capable on Teensy)
  attachInterrupt(digitalPinToInterrupt(6), MotorISR_RIGHT_FRONT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(8), MotorISR_LEFT_FRONT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(32), MotorISR_LEFT_REAR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(36), MotorISR_RIGHT_REAR, CHANGE);

  last_time = millis();
}

// -----------------------------------------------------------------------------
// Main loop: calculate odometry and publish it as a nav_msgs/Odometry message.
void loop() {
  nh.spinOnce();
  
  unsigned long now = millis();
  double elapsed = (now - last_time) / 1000.0;  // elapsed time in seconds
  
  if (elapsed >= PUBLISH_PERIOD) {
    // Compute average encoder counts for each side:
    long left_count = (encoder_data[LEFT_FRONT] + encoder_data[LEFT_REAR]) / 2;
    long right_count = (encoder_data[RIGHT_FRONT] + encoder_data[RIGHT_REAR]) / 2;
    
    // Calculate difference from last update:
    long d_left = left_count - last_left_count;
    long d_right = right_count - last_right_count;
    
    last_left_count = left_count;
    last_right_count = right_count;
    
    // Convert tick differences to distances (meters)
    // Each tick corresponds to a rotation: tick2rad * WHEEL_RADIUS gives linear distance per tick.
    double dist_left = d_left * TICK2RAD * WHEEL_RADIUS;
    double dist_right = d_right * TICK2RAD * WHEEL_RADIUS;
    
    // Differential drive kinematics:
    double d_center = (dist_left + dist_right) / 2.0;
    double d_theta = (dist_right - dist_left) / WHEEL_SEPARATION;
    
    // Update pose using midpoint integration
    x_pos += d_center * cos(theta + d_theta / 2.0);
    y_pos += d_center * sin(theta + d_theta / 2.0);
    theta += d_theta;
    
    // Compute velocities
    double vx = d_center / elapsed;
    double vth = d_theta / elapsed;
    
    // Fill in the odometry message
    odom.header.stamp = nh.now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_pos;
    odom.pose.pose.position.y = y_pos;
    odom.pose.pose.position.z = 0.0;
    
    // Create a quaternion from yaw
    // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    odom.pose.pose.orientation = createQuaternionFromYaw(theta);
    
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vth;
    
    // Publish odometry
    odom_pub.publish(&odom);
    
    // Prepare and broadcast the corresponding TF transform
    odom_tf.header.stamp = nh.now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = x_pos;
    odom_tf.transform.translation.y = y_pos;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = createQuaternionFromYaw(theta);
    tf_broadcaster.sendTransform(odom_tf);
    
    last_time = now;
  }
  
  delay(10);
}
