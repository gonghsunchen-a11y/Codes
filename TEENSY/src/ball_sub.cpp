#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

void readMainCore();

//ball
float vx;float vy;float ballDeg;int ballDist;

float finalVx;
float finalVy;



void readMainCore() {

  while (Serial8.available()) {

    // ===== DATA FRAME =====
    if (Serial8.available() < 8) return;

    if (Serial8.read() != 0xAA) continue;
    if (Serial8.read() != 0xAA) continue;

    uint8_t buffer[8];
    buffer[0] = 0xAA;
    buffer[1] = 0xAA;

    for (int i = 2; i < 8; i++) {
      buffer[i] = Serial8.read();
    }

    // check end
    if (buffer[7] != 0xEE) continue;

    // checksum
    uint8_t sum = 0;
    for (int i = 2; i <= 5; i++) {
      sum += buffer[i];
    }
    if (sum != buffer[6]) continue;

    // decode
    int16_t vx_i = (buffer[3] << 8) | buffer[2];
    int16_t vy_i = (buffer[5] << 8) | buffer[4];

    vx = vx_i;
    vy = vy_i;

    return; // 一次只處理一包
  }
}

void setup(){
  Robot_Init();
  Serial2.begin(115200);
  
}


void loop(){
  readBNO085Yaw();
  readMainCore();
  if (fabs(gyroData.pitch) > 15) {
    finalVx= 0;finalVy = 0;
    control.robot_heading = 90; 
    Vector_Motion(0, 0); 
    Serial.println("ROBOT PICKED UP - ALL STATES RESET");
    return; 
  }
  
  /*if(onLine){
    finalVx = lineVx;
    finalVy = lineVy;
  }
  else{
      finalVx = vx;
      finalVy = vy;
  }*/
  finalVx = vx;
  finalVy = vy;
  Vector_Motion(finalVx,finalVy);
  
  //Serial.print("vx= ");Serial.println(finalVx);
  //Serial.print("vy= ");Serial.println(finalVy);
  
}