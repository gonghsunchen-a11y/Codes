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

void drawState(const char *text){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println(text);
  display.display();
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
  showBallDirection();
  
  
  if(ballData.valid){
  float moving_degree = ballData.angle;
  float ballspeed = 30;

  if(ballData.dist > 15){
    // 訊號弱，球遠，直接追球
    moving_degree = ballData.angle;
    ballspeed = 30;
    if(ballData.angle >= 80 && ballData.angle <= 100)
    {
      moving_degree = 90;
    }  
  }
  else{
    // 訊號強，球近，開始繞球
    ballspeed = 30;

    if(ballData.angle >= 80 && ballData.angle <= 100){
      moving_degree = 90;
      ballspeed = 50;
    }
    else if(ballData.angle > 100 && ballData.angle < 180){
      moving_degree = ballData.angle + 45;
    }
    else if(ballData.angle >= 180 && ballData.angle <= 270){
      moving_degree = ballData.angle + 90;
    }
    else if(ballData.angle < 80 && ballData.angle > 0){
      moving_degree = ballData.angle - 45;
    }
    else if(ballData.angle <= 360 && ballData.angle > 270){
      moving_degree = ballData.angle - 90;
    }
  }

  if(moving_degree < 0) moving_degree += 360;
  if(moving_degree >= 360) moving_degree -= 360;

  ballData.Vx = (int)round(ballspeed * cos(moving_degree * DtoR_const));
  ballData.Vy = (int)round(ballspeed * sin(moving_degree * DtoR_const));

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
  else {
    uint8_t packet[8] = {0xAA,0xAA,0,0,0,0,0,0xEE};
    Serial8.write(packet, 8);
    Serial.println("0 ");
  }
}
