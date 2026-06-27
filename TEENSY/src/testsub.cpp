#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <math.h>
#include <Robot.h>

int offset;

void setup(){
    Robot_Init();
    Serial2.begin(115200);
}



void loop(){
  readBNO085Yaw();
  while(Serial8.available()){
    if(Serial8.available() < 3) return;

    if(Serial8.read() != 0xAA) continue;

    uint8_t buffer[3];
    buffer[0] = 0xAA;
    int8_t ofst  = buffer[1];
    buffer[2] = 0xEE;

    offset = ofst;
    return;
  }
  Serial.println(offset);
  Vector_Motion(0,0,offset);
  
} 
