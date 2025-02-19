#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>
#include <math.h>
#include <geometry_msgs/msg/transform_stamped.h>

// Constant for PI
#define PI 3.1415926535897932384626433832795

// Global odometry variables:
// odom_pose holds [x, y, theta] (position and heading in radians)
// odom_vel holds [linear_velocity, 0, angular_velocity]
float odom_pose[3] = {0.0, 0.0, 0.0};
double odom_vel[3] = {0.0, 0.0, 0.0};

// Initialize odometry variables
void initOdom(void) {
  odom_pose[0] = 0.0;
  odom_pose[1] = 0.0;
  odom_pose[2] = 0.0;
  odom_vel[0] = 0.0;
  odom_vel[1] = 0.0;
  odom_vel[2] = 0.0;
}

// Stub functions for motor state and velocity initialization
void initMotorState(void) {
  // Not needed in this simplified version
}

void initMotorVel(void) {
  // Not needed in this simplified version
}

// Calculate odometry based on sensor data.
// For this example, we simulate constant velocities.
// Replace these simulated values with your actual sensor readings.
bool calcOdometry(double dt) {
  float v = 0.1;      // Linear velocity in m/s (simulated)
  float omega = 0.05; // Angular velocity in rad/s (simulated)

  // Calculate displacement and heading change
  float delta_s = v * dt;
  float delta_theta = omega * dt;

  // Update pose using the current heading
  odom_pose[0] += delta_s * cos(odom_pose[2]);
  odom_pose[1] += delta_s * sin(odom_pose[2]);
  odom_pose[2] += delta_theta;

  // Update velocity information
  odom_vel[0] = v;
  odom_vel[2] = omega;
  return true;
}

// In this simplified implementation, updateOdometry() does nothing additional.
void updateOdometry(void) {
  // The odom_pose and odom_vel arrays are updated in calcOdometry().
}

// Update the TF message based on the current odometry.
// This converts the [x, y, theta] pose to a TransformStamped message.
void updateTF(geometry_msgs__msg__TransformStamped &tf_msg) {
  tf_msg.transform.translation.x = odom_pose[0];
  tf_msg.transform.translation.y = odom_pose[1];
  tf_msg.transform.translation.z = 0.0; // Planar motion assumed

  float yaw = odom_pose[2];
  tf_msg.transform.rotation.x = 0.0;
  tf_msg.transform.rotation.y = 0.0;
  tf_msg.transform.rotation.z = sin(yaw / 2.0);
  tf_msg.transform.rotation.w = cos(yaw / 2.0);
}

#endif // ROBOT_CONFIG_H
