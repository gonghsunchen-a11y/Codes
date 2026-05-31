#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <math.h>
#include <Robot.h>


void setup(){
    Robot_Init();
    Serial2.begin(115200);
}



void loop(){
  readBNO085Yaw();
  Vector_Motion(0,30,0,1,0);
  
} 
