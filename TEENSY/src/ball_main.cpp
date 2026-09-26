#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

void setup(){
  Robot_Init();
}
void showBallDirection() {
  display.clearDisplay();

  int cx = 64;
  int cy = 32;
  int r = 24;

  display.drawCircle(cx, cy, r, SSD1306_WHITE);

  if (ballData.valid) {
    float rad = (ballData.angle+180) * DtoR_const;

    int x = cx + cos(rad) * r;
    int y = cy - sin(rad) * r;

    display.drawLine(cx, cy, x, y, SSD1306_WHITE);
    display.fillCircle(x, y, 2, SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("A:");
    display.print(ballData.angle);

    display.setCursor(0, 10);
    display.print("D:");
    display.print(ballData.dist);
  } else {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("NO BALL");
  }

  display.display();
}

void loop(){
  readBNO085Yaw();
  ballsensor();
  showBallDirection();
  
  
  if(ballData.valid){
    float moving_degree = ballData.angle;
    if(ballData.angle>80&& ballData.angle<100){
      moving_degree = 90;
    }
  ballData.Vx = (int)round(50 * cos(moving_degree * DtoR_const));
  ballData.Vy = (int)round(50 * sin(moving_degree * DtoR_const));\

  Serial.print("ang = ");Serial.print(ballData.angle);
  Serial.print("dis = ");Serial.println(ballData.dist);
  Serial.print("vx = ");Serial.print(ballData.Vx);
  Serial.print("vy = ");Serial.println(ballData.Vy);
  
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
  //delay(100);
  /*static uint32_t lastDisplayTime = 0;
    //轉成弧度
    float moving_degree = ballData.angle;
    float offset = 0;

  
    float ballspeed = constrain(map(ballData.dist, 25, 55, 15, 18),15, 18);
   
    
    if(ballData.dist >= 17){
      moving_degree = ballData.angle;
      offset = 0;
    }
    else{
      float angleError = fabs(ballData.angle - 90);
      float angleFactor = 1.0 - constrain(abs(angleError) / 90.0, 0.0, 1.0);
      //float offsetFactor = constrain(angleError / 90.0, 0.0, 1.0); 
      ballspeed = ballspeed * (0.6 + 0.4 * angleFactor);
      float side;

      if(ballData.angle >= 80 && ballData.angle <= 100){
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
  }*/
}