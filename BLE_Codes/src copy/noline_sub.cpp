#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

void readMainCore();

//ball
float vx;float vy;float ballDeg;int ballDist;
float omega;
uint8_t goal_valid = 0x00;  // ✅ 拉到全域

float finalVx;
float finalVy;
void readMainCore(){
    static uint8_t buffer[11];
    static int index = 0;
    while(Serial8.available() > 0){
        uint8_t b = Serial8.read();
        if(index == 0 || index == 1){
            if(b == 0xAA) buffer[index++] = b;
            else index = 0;
            continue;
        }
        buffer[index++] = b;
        if(index == 11){
            index = 0;
            if(buffer[10] != 0xEE) continue;
            uint8_t sum = 0;
            for(int i = 2; i <= 8; i++) sum += buffer[i];
            if(sum != buffer[9]) continue;
            vx         = (int16_t)((buffer[3] << 8) | buffer[2]);
            vy         = (int16_t)((buffer[5] << 8) | buffer[4]);
            omega      = (int16_t)((buffer[7] << 8) | buffer[6]);
            goal_valid =  buffer[8];
        }
    }
}

void setup(){
  Robot_Init();
  Serial2.begin(115200);
}

void loop(){
  readMainCore();
  readBNO085Yaw();
  
  finalVx = vx;
  finalVy = vy;
  float omg = -omega/2000;
    Serial.println(omg,3);
  if(goal_valid == 0x00){
      Vector_Motion(finalVx, finalVy, 0, 1,0);
  }
  else{
      Vector_Motion(finalVx, finalVy, omg, 0,0);
  }
}