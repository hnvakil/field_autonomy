#include <Servo.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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
float speeds[2] = {0.0, 0.0}; //0 = left tread speed, 1 = right tread speed

float forwardBack = 0.0; // 0 to 1 forwards, 0 to -1 back
float leftRight = 0.0; // 0 to 1 right, 0 to -1 left

float leftTrackSpeed;
float rightTrackSpeed;

int leftStickVal;

bool printDebugging = true;

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = true;
float serialInput[2]; //0: linear, 1: angular



void setup() {
  Serial.begin(9600);
  leftMotor.attach(9);  // attaches the esc on pin 9 to the servo object
  rightMotor.attach(10); // attaches the esc on pin 10 to the servo object
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  leftStickVal = getLeftStickPos();
  if (leftStickVal == 2){
    digitalWrite(LED_BUILTIN, HIGH);
    if (printDebugging) {
      Serial.println("In serial listening mode");
    }
    Serial.println("before recv");
    recvWithEndMarker();
    Serial.println("after recv");
    showNewData();
    Serial.println("after showNew");
    if (printDebugging){
      Serial.print("linear val: ");
      Serial.print(serialInput[0]);
      Serial.print("   angular val: ");
      Serial.println(serialInput[1]);
    }
    getSpeedsSerial(serialInput[0], serialInput[1]);
    //Serial.println(serialInput[0]);
    if (printDebugging){
      Serial.print("  left tread: ");
      Serial.print(speeds[0]);
      Serial.print("  right tread: ");
      Serial.print(speeds[1]);
    }
    writeSpeeds(speeds[0], speeds[1]);
    delay(500); //REMOVE ONLY FOR DEBUGGING
    if (printDebugging){
      Serial.println("");
    }
  } else if (leftStickVal == 1){
    digitalWrite(LED_BUILTIN, LOW);
    getStickPositions();
    getSpeedsStick(joystickPositions[0], joystickPositions[1]);
    writeSpeeds(speeds[0], speeds[1]);
    //writeSpeeds(0.0,0.0);

    if (printDebugging) {
      Serial.print("  | stick up down ");
      Serial.print(joystickPositions[0]);
      Serial.print("  | stick leftRight ");
      Serial.print(joystickPositions[1]);

      Serial.print("  | left tread speed ");
      Serial.print(speeds[0]);
      Serial.print("  | right tread speed ");
      Serial.print(speeds[1]);
      
      Serial.println("");
    }
    delay(50);
  } else {
    if (printDebugging){
      Serial.println("In passive mode");
    }
    writeSpeeds(0.0,0.0);
  }
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '>';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
        char* velVal = strtok(receivedChars, ",");
        char* firstValChar = velVal;
        velVal = strtok(NULL, ",");
        char* secondValChar = velVal;
        float firstFloat = atof(firstValChar);
        float secondFloat = atof(secondValChar);
        Serial.print("first float: ");
        Serial.print(firstFloat);
        Serial.print(" | second float: ");
        Serial.println(secondFloat);

        serialInput[0] = firstFloat;
        serialInput[1] = secondFloat;
        newData = false;
    }
}

void getSpeedsSerial(float linear, float angular){
  float leftTread = linear - angular;
  float rightTread = linear + angular;
  speeds[0] = leftTread;
  speeds[1] = rightTread;
}

int getLeftStickPos(){
  ch3Value = readChannel(CH3, -100, 100, 0);
  //Serial.print("   ch3 val: ");
  //Serial.println(ch3Value);
  if (ch3Value > 1800){
    return 2;
  } else if (ch3Value > 1100){
    return 1;
  } else {
    return 0;
  }
}

void getSpeedsStick(float forwardBack, float leftRight){
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