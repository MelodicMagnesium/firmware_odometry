/*
  Minimal Encoder Count Test for a Four‑Wheel Robot

  This sketch reads quadrature encoder data from four wheels and prints the counts 
  to the serial monitor every 500 ms.

  Pin configuration (from your robot_config.h):
    RIGHT_FRONT_ENC_A = 6, RIGHT_FRONT_ENC_B = 7
    LEFT_FRONT_ENC_A  = 8, LEFT_FRONT_ENC_B  = 9
    LEFT_REAR_ENC_A   = 32, LEFT_REAR_ENC_B  = 33
    RIGHT_REAR_ENC_A  = 36, RIGHT_REAR_ENC_B = 37

  The wheel indices are defined as follows:
    RIGHT_FRONT = 0
    LEFT_FRONT  = 1
    LEFT_REAR   = 2
    RIGHT_REAR  = 3
*/
#include <Arduino.h>

#define RIGHT_FRONT 0
#define LEFT_FRONT  1
#define LEFT_REAR   2
#define RIGHT_REAR  3

// Encoder pin definitions:
#define RIGHT_FRONT_ENC_A 6
#define RIGHT_FRONT_ENC_B 7

#define LEFT_FRONT_ENC_A 8
#define LEFT_FRONT_ENC_B 9

#define LEFT_REAR_ENC_A 32
#define LEFT_REAR_ENC_B 33

#define RIGHT_REAR_ENC_A 36
#define RIGHT_REAR_ENC_B 37

// Global encoder counts for the four wheels
volatile long encoder_data[4] = {0, 0, 0, 0};

// ISR for Right Front wheel encoder
void encoderISR_RightFront() {
  int b = digitalRead(RIGHT_FRONT_ENC_B);
  if (digitalRead(RIGHT_FRONT_ENC_A) == b)
    encoder_data[RIGHT_FRONT]++;
  else
    encoder_data[RIGHT_FRONT]--;
}

// ISR for Left Front wheel encoder
void encoderISR_LeftFront() {
  int b = digitalRead(LEFT_FRONT_ENC_B);
  if (digitalRead(LEFT_FRONT_ENC_A) == b)
    encoder_data[LEFT_FRONT]++;
  else
    encoder_data[LEFT_FRONT]--;
}

// ISR for Left Rear wheel encoder
void encoderISR_LeftRear() {
  int b = digitalRead(LEFT_REAR_ENC_B);
  if (digitalRead(LEFT_REAR_ENC_A) == b)
    encoder_data[LEFT_REAR]++;
  else
    encoder_data[LEFT_REAR]--;
}

// ISR for Right Rear wheel encoder
void encoderISR_RightRear() {
  int b = digitalRead(RIGHT_REAR_ENC_B);
  if (digitalRead(RIGHT_REAR_ENC_A) == b)
    encoder_data[RIGHT_REAR]++;
  else
    encoder_data[RIGHT_REAR]--;
}

void setup() {
  Serial.begin(115200);  // Open serial monitor at 115200 baud

  // Configure encoder pins as inputs with internal pull-ups
  pinMode(RIGHT_FRONT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_FRONT_ENC_B, INPUT_PULLUP);
  
  pinMode(LEFT_FRONT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_FRONT_ENC_B, INPUT_PULLUP);
  
  pinMode(LEFT_REAR_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_REAR_ENC_B, INPUT_PULLUP);
  
  pinMode(RIGHT_REAR_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_REAR_ENC_B, INPUT_PULLUP);
  
  // Attach interrupts to the encoder A channels.
  // digitalPinToInterrupt() ensures compatibility with various Arduino boards.
  attachInterrupt(digitalPinToInterrupt(RIGHT_FRONT_ENC_A), encoderISR_RightFront, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_FRONT_ENC_A), encoderISR_LeftFront, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_REAR_ENC_A), encoderISR_LeftRear, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_REAR_ENC_A), encoderISR_RightRear, CHANGE);
}

void loop() {
  // Print encoder counts for all four wheels every 500 milliseconds
  Serial.print("Right Front: ");
  Serial.print(encoder_data[RIGHT_FRONT]);
  Serial.print(" | Left Front: ");
  Serial.print(encoder_data[LEFT_FRONT]);
  Serial.print(" | Left Rear: ");
  Serial.print(encoder_data[LEFT_REAR]);
  Serial.print(" | Right Rear: ");
  Serial.println(encoder_data[RIGHT_REAR]);
  
  delay(500);
}
