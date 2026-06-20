#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define CMD_ATTACK 0xAA
#define CMD_LINECAL_START 0xCC
#define CMD_LINECAL_SAVE 0xEE
#define CMD_LINECAL_DONE 0xDD

#define RIGHT_SLOW_X 65
#define RIGHT_STOP_X 80
#define LEFT_SLOW_X -65
#define LEFT_STOP_X -80
#define SIDE_SLOW_EXP 2.0f

enum MainState { READY, SCANNING, ATTACK };
MainState state = READY;

bool btnPressed(int pin){
  static unsigned long last[40] = {0};
  if(digitalRead(pin) == LOW && millis() - last[pin] > 200){
    last[pin] = millis();
    return true;
  }
  return false;
}

void setup(){
  delay(3000);  
  Robot_Init();

}

void showBallDirection() {
  display.clearDisplay();

  int cx = 64;
  int cy = 32;
  int r = 24;

  display.drawCircle(cx, cy, r, SSD1306_WHITE);

  if(maixPosData.valid&&maixPosData.ball_found){
    float rad = (maixPosData.ball_angle+180) * DtoR_const;

    int x = cx + cos(rad) * r;
    int y = cy - sin(rad) * r;

    display.drawLine(cx, cy, x, y, SSD1306_WHITE);
    display.fillCircle(x, y, 2, SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print("A:");
    display.print(maixPosData.ball_angle);
  }
  else if(ballData.valid){
    float rad = (ballData.angle+180) * DtoR_const;

    int x = cx + cos(rad) * r;
    int y = cy - sin(rad) * r;

    display.drawLine(cx, cy, x, y, SSD1306_WHITE);
    display.fillCircle(x, y, 2, SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print("A:");
    display.print(ballData.angle);
  } else {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("NO BALL");
  }
  display.display();
}

void drawState(const char *text){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println(text);
  display.display();
}

// ------------------ Omni Mirror ------------------
void applyOmniEdgeBrake(int16_t &vx){
  if(!maixPosData.valid ){
    return;
  }

  float x = maixPosData.x;
  if(vx>0){  if(x >= RIGHT_STOP_X){
      vx = 0;
      return;
    }

    if(x >= RIGHT_SLOW_X){
      float t = (x - RIGHT_SLOW_X) / (RIGHT_STOP_X - RIGHT_SLOW_X);
      t = constrain(t, 0.0f, 1.0f);

      float scale = exp(-SIDE_SLOW_EXP * t);
      vx = (int16_t)round(vx * scale);
    }

    return;
  }
  if (vx < 0) {
    if (x <= LEFT_STOP_X) {
      vx = 0;
      return;
    }

    if (x <= LEFT_SLOW_X) {
      float t = (LEFT_SLOW_X - x) / (LEFT_SLOW_X - LEFT_STOP_X);
      t = constrain(t, 0.0f, 1.0f);

      float scale = exp(-SIDE_SLOW_EXP * t);
      vx = (int16_t)round(vx * scale);
    }

    return;
  }
}

void loop(){
  
  readBNO085Yaw();
  ballsensor();
  readMaix();

  if(state == READY || state == SCANNING){
    if(btnPressed(BTN_ENTER) && state == READY){
      state = SCANNING;
      Serial8.write(CMD_LINECAL_START);
      drawState("SCANNING");
    }

    if(btnPressed(BTN_ESC) && state == SCANNING){
      Serial8.write(CMD_LINECAL_SAVE);
      state = READY;

      unsigned long t = millis();
      while(millis() - t < 2000){
        if(Serial8.available() && Serial8.read() == CMD_LINECAL_DONE) break;
      }

      drawState("SAVED\nREADY");
      delay(500);
    }

    if(btnPressed(BTN_UP) && state == READY){
      state = ATTACK;
      Serial8.write(CMD_ATTACK);
      drawState("ATTACK");
    }

    if(state == READY){
      drawState("READY");
    }
    return;
  }

  
  //showBallDirection();

  int16_t vx = 0;
  int16_t vy = 0;
  
  
  if(ballData.valid){
    if(maixPosData.valid && maixPosData.ball_found){
      float moving_degree = maixPosData.ball_angle;
      float offset = 0;
      float ballspeed = constrain(map(maixPosData.ball_dist, 70, 85, 35, 50), 35, 50);
      //float ballspeedVx = constrain(map(maixPosData.ball_dist, 70, 85, 30, 50), 30, 50);
      //float ballspeedVy = constrain(map(maixPosData.ball_dist, 70, 85, 25, 50), 25, 50);

      //Serial.println(maixPosData.ball_dist);
      if(maixPosData.ball_angle >= 80 && maixPosData.ball_angle <= 100){
        moving_degree = 90;
        ballspeed = 60;
      }
      else if(maixPosData.ball_angle > 100 && maixPosData.ball_angle < 180){
        float offsetRatio = exp(-0.08 * (maixPosData.ball_dist - 70));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 20 + 70 * offsetRatio;
        moving_degree = maixPosData.ball_angle + offset;
        //float angleError = fabs(ballData.angle - 90);
        //float smoothWeight = constrain(angleError / 25.0f, 0.0f, 1.0f);
      }
      else if(maixPosData.ball_angle >= 180 && maixPosData.ball_angle <= 270){
        float offsetRatio = exp(-0.08 * (maixPosData.ball_dist - 70));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 90 * offsetRatio;
        Serial.print(" offset=");Serial.print(offset);
        moving_degree = maixPosData.ball_angle + offset;
      }
      else if(maixPosData.ball_angle < 80 && maixPosData.ball_angle >= 0){
        float offsetRatio = exp(-0.08 * (maixPosData.ball_dist - 70));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 20 + 70 * offsetRatio;
        moving_degree = maixPosData.ball_angle - offset;
      }
      else if(maixPosData.ball_angle < 360 && maixPosData.ball_angle > 270){
        float offsetRatio = exp(-0.08 * (maixPosData.ball_dist - 70));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 90*offsetRatio;
        moving_degree = maixPosData.ball_angle - offset;
      }
    
      if(moving_degree < 0) moving_degree += 360;
      if(moving_degree >= 360) moving_degree -= 360;

      Serial.print(" ball=");Serial.print(maixPosData.ball_angle);
      //Serial.print(" balldist=");Serial.println(ballData.dist);
      Serial.print(" balldist=");Serial.print(maixPosData.ball_dist);
      //Serial.print(" move=");Serial.println(moving_degree);
    
      float angleError = fabs(ballData.angle - 90);
      float vxWeight = constrain(angleError / 20.0f, 0.3f, 1.0f);
      vx = (int)round(vx* vxWeight);

      vx = (int16_t)round(ballspeed * cos(moving_degree * DtoR_const));
      vy = (int16_t)round(ballspeed * sin(moving_degree * DtoR_const));
    }
    else{
      float moving_degree = ballData.angle;
      float ballspeed = constrain(map(ballData.dist, 5, 1, 50, 80), 50, 80);
      if(ballData.dist>=6){ballspeed =50;}
      else{ballspeed=80;}
      Serial.println(ballData.dist);
      
      if(moving_degree < 0) moving_degree += 360;
      if(moving_degree >= 360) moving_degree -= 360;

      vx = (int16_t)round(ballspeed * cos(moving_degree * DtoR_const));
      vy = (int16_t)round(ballspeed * sin(moving_degree * DtoR_const));
    }
  }
  else{
    vx = 0;
    vy = 0;
  }
  applyOmniEdgeBrake(vx);
   /* if(maixPosData.valid && maixPosData.ball_found){
      float moving_degree = maixPosData.ball_angle;
      float ballspeed = 30;
      Serial.println(maixPosData.ball_angle);
      if(maixPosData.ball_angle >= 80 && maixPosData.ball_angle <= 100){
        moving_degree = 90;
      }
      else if(maixPosData.ball_angle > 100 &&maixPosData.ball_angle < 180){
        moving_degree = maixPosData.ball_angle + 45;
      }
      else if(maixPosData.ball_angle >= 180 && maixPosData.ball_angle <= 270){
        moving_degree = maixPosData.ball_angle + 90;
      }
      else if(maixPosData.ball_angle < 80 &&maixPosData.ball_angle > 0){
        moving_degree = maixPosData.ball_angle - 45;
      }
      else if(maixPosData.ball_angle <= 360 && maixPosData.ball_angle > 270){
        moving_degree = maixPosData.ball_angle - 90;
      }

      if(moving_degree < 0) moving_degree += 360;
      if(moving_degree >= 360) moving_degree -= 360;

      vx = (int16_t)round(ballspeed * cos(moving_degree * DtoR_const));
      vy = (int16_t)round(ballspeed * sin(moving_degree * DtoR_const));
  
    }
  else{
    vx=0;vy=0;
  }*/
    /*
  else if(ballData.valid){
    IRchase(vx,vy);
    //Serial.print(" Ball angle=");
    //Serial.print(ballData.angle);
  }
  else{
    vx=0;vy=0;
    Serial.println(" NO BALL");
  }*/
  Serial.print(" vx=");Serial.print(vx);
  Serial.print(" vy=");Serial.println(vy);
  uint8_t packet[8];

  packet[0] = 0xAA;
  packet[1] = 0xAA;
  packet[2] = vx & 0xFF;
  packet[3] = (vx >> 8) & 0xFF;
  packet[4] = vy & 0xFF;
  packet[5] = (vy >> 8) & 0xFF;

  uint8_t sum = 0;
  for(int i = 2; i <= 5; i++){
    sum += packet[i];
  }

  packet[6] = sum;
  packet[7] = 0xEE;
  Serial8.write(packet, 8);
}
