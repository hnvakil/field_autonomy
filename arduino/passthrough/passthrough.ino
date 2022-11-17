#include <Servo.h>
#include <math.h>

#define CH1 3
#define CH2 5
#define CH3 6

Servo rightMotor;  // actually right esc
Servo leftMotor; // actually left esc

int ch1Value;
int ch2Value;
int ch3Value;
float ch1Norm;
float ch2Norm;
float upDown;
float leftRightStick;
float joystickPositions[2]; //0 = upDown, 1 = leftRight
float speeds[2]; //0 = left tread speed, 1 = right tread speed

float forwardBack = 0.0; // 0 to 1 forwards, 0 to -1 back
float leftRight = 0.0; // 0 to 1 right, 0 to -1 left

float leftTrackSpeed;
float rightTrackSpeed;


void setup() {
  Serial.begin(9600);
  leftMotor.attach(9);  // attaches the esc on pin 9 to the servo object
  rightMotor.attach(10); // attaches the esc on pin 10 to the servo object
}

void loop() {
  getStickPositions();
  Serial.print("  | stick up down ");
  Serial.print(joystickPositions[0]);
  Serial.print("  | stick leftRight ");
  Serial.print(joystickPositions[1]);

  getSpeeds(joystickPositions[0], joystickPositions[1]);
  Serial.print("  | speed front back ");
  Serial.print(speeds[0]);
  Serial.print("  | speed left right ");
  Serial.print(speeds[1]);

  writeSpeeds(speeds[0], speeds[1]);
  Serial.println("");

  delay(50);
  //writeSpeeds(0.0,0.0);

  //delay(500);
}

void getSpeeds(float forwardBack, float leftRight){
  leftTrackSpeed = forwardBack + leftRight;
  rightTrackSpeed = forwardBack - leftRight;
  if (abs(leftTrackSpeed) > 1){
    leftTrackSpeed = leftTrackSpeed / abs(leftTrackSpeed);
  }
  if (abs(rightTrackSpeed) > 1){
    rightTrackSpeed = rightTrackSpeed / abs(rightTrackSpeed);
  }
  speeds[0] = leftTrackSpeed;
  speeds[1] = rightTrackSpeed;
}

void writeSpeeds(float leftSpeed, float rightSpeed){
  if (leftSpeed > 0) {
    leftMotor.writeMicroseconds(leftSpeed * -470 + 1270);
  } else if (leftSpeed < 0) {
    leftMotor.writeMicroseconds(leftSpeed * -830 + 1350);
  } else {
    leftMotor.writeMicroseconds(1309);
  }
  if (rightSpeed > 0) {
    rightMotor.writeMicroseconds(rightSpeed * 830 + 1350);
  } else if (rightSpeed < 0) {
    rightMotor.writeMicroseconds(rightSpeed * 470 + 1270);
  } else {
    rightMotor.writeMicroseconds(1309);
  }
}

void getStickPositions(){
  ch1Value = readChannel(CH1, -100, 100, 0);
  ch2Value = readChannel(CH2, -100, 100, 0);
  ch3Value = readChannel(CH3, -100, 100, 0);
  
  ch1Norm = ch1Value - 1500;
  ch2Norm = ch2Value - 1455;
  if (ch1Norm > 0){
    ch1Norm = ch1Norm / 488;
  } else {
    ch1Norm = ch1Norm / 506;
  }
  if (ch2Norm > 0){
    ch2Norm = ch2Norm / 533;
  } else {
    ch2Norm = ch2Norm / 461;
  }
  upDown = ch2Norm - ch1Norm;
  leftRightStick = -1 * ch1Norm - ch2Norm;
  if (abs(upDown) > 1){
    upDown = upDown / abs(upDown);
    leftRightStick = leftRightStick / abs(upDown);
  }
  if (abs(leftRightStick) > 1){
    upDown = upDown / abs(leftRightStick);
    leftRightStick = leftRightStick / abs(leftRightStick);
  }
  
  if (abs(upDown) < 0.06){
    upDown = 0.0;
  }
  if (abs(leftRightStick) < 0.06) {
    leftRightStick = 0.0;
  }
  if (abs(upDown) < 0.06 && abs(leftRightStick) < 0.2){
    leftRightStick = 0.0;
  }
  joystickPositions[0] = upDown;
  joystickPositions[1] = leftRightStick;
}

int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  //if (ch < 100) return defaultValue;
  //return map(ch, 1000, 2000, minLimit, maxLimit);
  return ch;
}