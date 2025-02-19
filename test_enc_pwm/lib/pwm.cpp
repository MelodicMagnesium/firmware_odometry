/*
  Motor Test Sketch for Teensy 4.1
  This sketch runs all four motors (assumed to be controlled via PWM and a direction pin)
  using the following pin assignments (from your robot_config.h):

    RIGHT_FRONT:
      PWM pin: 2
      Direction pin: 23

    LEFT_FRONT:
      PWM pin: 3
      Direction pin: 22

    LEFT_REAR:
      PWM pin: 4
      Direction pin: 40

    RIGHT_REAR:
      PWM pin: 5
      Direction pin: 41

  The sketch cycles through running the motors forward, stopping, then reversing.
*/
#include <Arduino.h>

#define RIGHT_FRONT 0
#define LEFT_FRONT  1
#define LEFT_REAR   2
#define RIGHT_REAR  3

// Motor pin definitions
const int RIGHT_FRONT_PWM = 2;
const int RIGHT_FRONT_DIR = 23;

const int LEFT_FRONT_PWM = 3;
const int LEFT_FRONT_DIR = 22;

const int LEFT_REAR_PWM = 4;
const int LEFT_REAR_DIR = 40;

const int RIGHT_REAR_PWM = 5;
const int RIGHT_REAR_DIR = 41;

// This function sets the motor's direction and PWM value.
// A positive PWM value drives the motor in one direction,
// and a negative value drives it in the opposite direction.
void MOTOR_PWM_CONTROL(int motor, int pwm) {
  // Constrain the PWM to the 8-bit range (-255 to 255)
  if (pwm > 255) pwm = 255;
  if (pwm < -255) pwm = -255;

  int pwmPin, dirPin;
  switch (motor) {
    case RIGHT_FRONT:
      pwmPin = RIGHT_FRONT_PWM;
      dirPin = RIGHT_FRONT_DIR;
      break;
    case LEFT_FRONT:
      pwmPin = LEFT_FRONT_PWM;
      dirPin = LEFT_FRONT_DIR;
      break;
    case LEFT_REAR:
      pwmPin = LEFT_REAR_PWM;
      dirPin = LEFT_REAR_DIR;
      break;
    case RIGHT_REAR:
      pwmPin = RIGHT_REAR_PWM;
      dirPin = RIGHT_REAR_DIR;
      break;
    default:
      return;
  }

  // Set the direction: for example, HIGH for forward and LOW for reverse.
  if (pwm > 0) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, pwm);
  } else if (pwm < 0) {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, -pwm);
  } else {
    // Stop the motor.
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) ;  // wait for serial monitor to open (optional on Teensy)

  // Set motor pins as outputs.
  pinMode(RIGHT_FRONT_PWM, OUTPUT);
  pinMode(RIGHT_FRONT_DIR, OUTPUT);

  pinMode(LEFT_FRONT_PWM, OUTPUT);
  pinMode(LEFT_FRONT_DIR, OUTPUT);

  pinMode(LEFT_REAR_PWM, OUTPUT);
  pinMode(LEFT_REAR_DIR, OUTPUT);

  pinMode(RIGHT_REAR_PWM, OUTPUT);
  pinMode(RIGHT_REAR_DIR, OUTPUT);

  // Initialize all motors to off.
  MOTOR_PWM_CONTROL(RIGHT_FRONT, 0);
  MOTOR_PWM_CONTROL(LEFT_FRONT, 0);
  MOTOR_PWM_CONTROL(LEFT_REAR, 0);
  MOTOR_PWM_CONTROL(RIGHT_REAR, 0);

  Serial.println("Motor Test Started");
}

void loop() {
  // Run all motors forward at PWM 150 for 3 seconds.
  Serial.println("Running motors FORWARD");
  for (int i = 0; i < 4; i++) {
    MOTOR_PWM_CONTROL(i, 150);
  }
  delay(3000);

  // Stop all motors for 1 second.
  Serial.println("Stopping motors");
  for (int i = 0; i < 4; i++) {
    MOTOR_PWM_CONTROL(i, 0);
  }
  delay(1000);

  // Run all motors in reverse at PWM -150 for 3 seconds.
  Serial.println("Running motors REVERSE");
  for (int i = 0; i < 4; i++) {
    MOTOR_PWM_CONTROL(i, -150);
  }
  delay(3000);

  // Stop all motors for 1 second.
  Serial.println("Stopping motors");
  for (int i = 0; i < 4; i++) {
    MOTOR_PWM_CONTROL(i, 0);
  }
  delay(1000);
}
