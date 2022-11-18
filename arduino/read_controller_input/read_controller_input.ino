#include <math.h>
/*
  Arduino FS-I6X Demo
  fsi6x-arduino-uno.ino
  Read output ports from FS-IA6B receiver module
  Display values on Serial Monitor
  
  Channel functions by Ricardo Paiva - https://gist.github.com/werneckpaiva/
  
  DroneBot Workshop 2021
  https://dronebotworkshop.com
*/

// Define Input Connections
#define CH1 3
#define CH2 5
#define CH3 6

// Integers to represent values from sticks and pots
int ch1Value;
int ch2Value;
int ch3Value;
float ch1Norm;
float ch2Norm;
float upDown;
float leftRight;
float joystickPositions[2]; //0 = upDown, 1 = leftRight

// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  //if (ch < 100) return defaultValue;
  //return map(ch, 1000, 2000, minLimit, maxLimit);
  return ch;
}

void setup(){
  // Set up serial monitor
  Serial.begin(115200);
  
  // Set all pins as inputs
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
}


void loop() {
  
  // Get values for each channel
  getStickPositions();
  Serial.print("joystick pos 0: ");
  Serial.print(joystickPositions[0]);
  Serial.print("  joystick pos 1: ");
  Serial.print(joystickPositions[1]);
  Serial.println("");
  
  
  
  
  delay(500);
}

void getStickPositions(){
  ch1Value = readChannel(CH1, -100, 100, 0);
  ch2Value = readChannel(CH2, -100, 100, 0);
  ch3Value = readChannel(CH3, -100, 100, 0);
  
  // Print to Serial Monitor
  Serial.print("Ch1: ");
  Serial.print(ch1Value);
  Serial.print(" | Ch2: ");
  Serial.print(ch2Value);
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
  leftRight = -1 * ch1Norm - ch2Norm;
  if (abs(upDown) > 1){
    upDown = upDown / abs(upDown);
    leftRight = leftRight / abs(upDown);
  }
  if (abs(leftRight) > 1){
    upDown = upDown / abs(leftRight);
    leftRight = leftRight / abs(leftRight);
  }
  Serial.print(" | upDown: ");
  Serial.print(upDown);
  Serial.print(" | leftRight: ");
  Serial.print(leftRight);
  joystickPositions[0] = upDown;
  joystickPositions[1] = leftRight;
  Serial.println("");
}