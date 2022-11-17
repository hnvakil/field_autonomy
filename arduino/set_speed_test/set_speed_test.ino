#include <Servo.h>

Servo right;  // create servo object to control a servo
Servo left;
// twelve servo objects can be created on most boards

float leftSpeed = 0.5;
float rightSpeed = -0.5;

void setup() {
  Serial.begin(9600);
  left.attach(9);  // attaches the servo on pin 9 to the servo object
  right.attach(10);
}

void loop() {
  writeSpeeds(leftSpeed, rightSpeed);
}

void writeSpeeds(float leftSpeed, float rightSpeed){
  if (leftSpeed > 0) {
    left.writeMicroseconds(leftSpeed * -470 + 1270);
  } else if (leftSpeed < 0) {
    left.writeMicroseconds(leftSpeed * -830 + 1350);
  } else {
    left.writeMicroseconds(1309);
  }
  if (rightSpeed > 0) {
    right.writeMicroseconds(rightSpeed * 830 + 1350);
  } else if (rightSpeed < 0) {
    right.writeMicroseconds(rightSpeed * 470 + 1270);
  } else {
    right.writeMicroseconds(1309);
  }
}