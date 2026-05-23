#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define possession_Threshold 150

//CAMERA
unsigned long lastCameraUpdate = 0;  // 記錄上次執行時間
const unsigned long interval = 50;  // 20Hz = 每50毫秒一次

//GOAL
//uint32_t lastTargetTime = 0;
//static float rotate = 0;
//static bool isRecovering = false; // 紀錄是否正在處理邊緣回彈

void setup(){
  Robot_Init();
  drawMessage("START");
}

void loop(){
  readBNO085Yaw();
  readBallCam();
  readussensor();
  static uint32_t lastDisplayTime = 0;
  if(ballData.valid){   //有球
    //Serial.print("Angle: "); Serial.println(ballData.angle);
    //Serial.print("Dist: "); Serial.println(ballData.dist);
    /*if (millis() - lastDisplayTime > 100) { // 每 0.1 秒更新一次螢幕
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      
      // 顯示指南針 (Heading) 輔助確認感測器是否正常
      display.setCursor(0, 20);
      //display.printf("pitch: %.1f", gyroData.pitch);
      display.printf("angle: %d\n", ballData.angle);

      display.printf("dist: %d\n",ballData.dist);
      display.display();
      lastDisplayTime = millis();
    }*/
    //轉成弧度
    float moving_degree = ballData.angle;
    float offset = 0;

  
    float ballspeed = constrain(map(ballData.dist, 25, 55, 40, 70),40, 70);
   
    
    if(ballData.dist >= 65){
      moving_degree = ballData.angle;
      offset = 0;
    }
    else{
      float angleError = fabs(ballData.angle - 90);
      float angleFactor = 1.0 - constrain(abs(angleError) / 90.0, 0.0, 1.0);
      //float offsetFactor = constrain(angleError / 90.0, 0.0, 1.0); 
      ballspeed = ballspeed * (0.6 + 0.4 * angleFactor);
      float side;

      if(ballData.angle >= 75 && ballData.angle <= 105){
        ballspeed = 50;
        side = 0;
        offset = 0;
        moving_degree = ballData.angle;
      }
      else if(ballData.angle > 105 && ballData.angle < 270){
        side = 1;
        float offsetRatio = exp(-1.5 * (ballData.dist - 55));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 90 * offsetRatio;
        //offset = 100 * offsetRatio * offsetFactor;
        moving_degree = ballData.angle + (offset * side);
      }

      else if(ballData.angle < 75 || ballData.angle >= 270){
        side = -1;
        float offsetRatio = exp(-1.5 * (ballData.dist - 55));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 90 * offsetRatio;
        //offset = 100 * offsetRatio * offsetFactor;
        moving_degree = ballData.angle + (offset * side);
      }
    
    }
    
    moving_degree = fmod(moving_degree + 360.0f, 360.0f) ;

    //計算vx vy
    ballData.Vx = (int)round(ballspeed * cos(moving_degree * DtoR_const));
    ballData.Vy = (int)round(ballspeed * sin(moving_degree * DtoR_const));
    
    if(ballData.dist <= 37 && ballData.angle <= 100 && ballData.angle >= 80){
      ballData.Vx = 0;
      ballData.Vy = 80;
      moving_degree = 90;
      offset = 0;
    }
    //Serial.print("f= ");Serial.println(usData.dist_f);
    //Serial.print("l= ");Serial.println(usData.dist_l);
    //Serial.print("b= ");Serial.println(usData.dist_b);
    //Serial.print("r= ");Serial.println(usData.dist_r);
    
    //右邊線
    if(usData.dist_r <= 22 ){if(ballData.Vx > 0)ballData.Vx = 0;}
    else if(usData.dist_r <= 25){if(ballData.Vx > 0)ballData.Vx *= 0.5; }
    else if(usData.dist_r <= 30){if(ballData.Vx > 0)ballData.Vx *= 0.7;}
    else{ballData.Vx = ballData.Vx;}
    //左邊線
    if(usData.dist_l <= 22 ){if(ballData.Vx < 0)ballData.Vx = 0;}
    else if(usData.dist_l <= 25){if(ballData.Vx < 0){ballData.Vx *= 0.5;}}
    else if(usData.dist_l <= 30){if(ballData.Vx < 0)ballData.Vx *= 0.7;}
    else{ballData.Vx = ballData.Vx;}
    //-------------------------------------------------------------------
    
    //前角落
    if(usData.dist_l <= 40){
      if(usData.dist_f <= 23){if(ballData.Vy > 0)ballData.Vy = 0;}
      else if(usData.dist_f <= 25){if(ballData.Vy > 0)ballData.Vy *= 0.6;}
      else if(usData.dist_f <= 27){if(ballData.Vy > 0)ballData.Vy *= 0.8;}
    }
    if(usData.dist_r <= 40){
      if(usData.dist_f <= 23){if(ballData.Vy > 0)ballData.Vy = 0;}
      else if(usData.dist_f <= 25){if(ballData.Vy > 0)ballData.Vy *= 0.6;}
      else if(usData.dist_f <= 27){if(ballData.Vy > 0)ballData.Vy *= 0.8;}
    }
    //後角落
    if(usData.dist_l <= 35){
      if(usData.dist_b <= 23){if(ballData.Vy < 0)ballData.Vy = 0;}
      else if(usData.dist_b <= 25){if(ballData.Vy < 0)ballData.Vy *= 0.6;}
      else if(usData.dist_b <= 27){if(ballData.Vy < 0)ballData.Vy *= 0.8;}
    }
    if(usData.dist_r <= 43){
      if(usData.dist_b<= 23){if(ballData.Vy < 0)ballData.Vy = 0;}
      else if(usData.dist_b <= 25){if(ballData.Vy < 0)ballData.Vy *= 0.6;}
      else if(usData.dist_b <= 27){if(ballData.Vy < 0)ballData.Vy *= 0.8;}
    }
    if(usData.dist_b<= 23){if(ballData.Vy < 0)ballData.Vy = 0;}
    if(usData.dist_f<= 23){if(ballData.Vy > 0)ballData.Vy = 0;}
    //-------------------------------------------------------------------
    
    Serial.print("angle= ");Serial.println(ballData.angle);
    Serial.print("dist= ");Serial.println(ballData.dist);
    Serial.print("moving= ");Serial.println(moving_degree);
    Serial.print("vx= ");Serial.println(ballData.Vx);
    Serial.print("vy= ");Serial.println(ballData.Vy);
    
    uint8_t packet[8];
    int16_t vx_i = (int16_t)(ballData.Vx);
    int16_t vy_i = (int16_t)(ballData.Vy);
    // Header
    packet[0] = 0xAA;
    packet[1] = 0xAA;
    // vx
    packet[2] = vx_i & 0xFF;
    packet[3] = (vx_i >> 8) & 0xFF;
    // vy
    packet[4] = vy_i & 0xFF;
    packet[5] = (vy_i >> 8) & 0xFF;
    // checksum
    uint8_t sum = 0;
    for(int i = 2; i <= 5; i++){
      sum += packet[i];
    }
    packet[6] = sum;
    // end
    packet[7] = 0xEE;

    Serial8.write(packet, 8);
  }
  else { //無球
    //drawMessage("NO BALL");
    uint8_t packet[8] = {0xAA,0xAA,0,0,0,0,0,0xEE};
    Serial8.write(packet, 8);
    Serial.println("0 ");
  }
}