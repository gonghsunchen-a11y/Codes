#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define CMD_ATTACK 0xAA
#define CMD_LINECAL_START 0xCC
#define CMD_LINECAL_SAVE 0xEE
#define CMD_LINECAL_DONE 0xDD

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

void Camchase(int16_t &vx, int16_t &vy){
  float moving_degree = maixPosData.ball_angle;
  float ballspeed = 30;

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
void IRchase(int16_t &vx, int16_t &vy){
  float moving_degree = ballData.angle;
  float ballspeed = 30;
  if(moving_degree < 0) moving_degree += 360;
  if(moving_degree >= 360) moving_degree -= 360;

  vx = (int16_t)round(ballspeed * cos(moving_degree * DtoR_const));
  vy = (int16_t)round(ballspeed * sin(moving_degree * DtoR_const));
}

void sendMovePacket(int16_t vx, int16_t vy){
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

void loop(){
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

  readBNO085Yaw();
  ballsensor();
  readMaix();
  showBallDirection();

  int16_t vx = 0;
  int16_t vy = 0;

  if(maixPosData.valid && maixPosData.ball_found){
    Camchase(vx,vy);
    Serial.print("X=");
    Serial.print(maixPosData.x);
    Serial.print(" Y=");
    Serial.print(maixPosData.y);
    Serial.print(" status=");
    Serial.println(maixPosData.status);  
    Serial.print(" Ball angle=");
    Serial.print(maixPosData.ball_angle);
    Serial.print(" dist=");
    Serial.println(maixPosData.ball_dist);
  }
  else if(ballData.valid){
    IRchase(vx,vy);
    Serial.print(" Ball angle=");
    Serial.print(ballData.angle);
  }
  else{
    vx=0;vy=0;
    Serial.println(" NO BALL");
  }
  sendMovePacket(vx,vy);
}
