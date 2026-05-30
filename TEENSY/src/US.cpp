#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>

#define TRIG 3
#define ECHO 8

float readRCW0001() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(ECHO, HIGH, 25000); // timeout 25ms

  if (duration == 0) {
    return -1; // 沒讀到
  }

  float distance_cm = duration * 0.0343 / 2.0;
  return distance_cm;
}


void setup(){
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}
void loop(){
  readBNO085Yaw();
  float d = readRCW0001();
  if(d<0){
    Serial.println("No echo");
  } else {
    Serial.print("Distance = ");
    Serial.print(d);
    Serial.println(" cm");
  }
  delay(50);
}