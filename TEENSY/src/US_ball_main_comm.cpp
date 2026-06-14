#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define CMD_ATTACK 0xAA
#define CMD_LINECAL_START 0xCC
#define CMD_LINECAL_SAVE 0xEE
#define CMD_LINECAL_DONE 0xDD

#define TRIG_F 2
#define ECHO_F 6
#define TRIG_R 3
#define ECHO_R 8
#define TRIG_B 4
#define ECHO_B 9
#define TRIG_L 5
#define ECHO_L 10

#define US_COUNT 4
#define US_INVALID_DISTANCE 999.0f
#define US_FILTER_ALPHA 0.35f
#define US_INVALID_LIMIT 3

#define SIDE_SLOW_DIST 60.0f
#define SIDE_STOP_DIST 30.0f
#define SIDE_DANGER_DIST 16.0f
#define SIDE_SLOW_EXP 2.0f
#define SIDE_PUSH_SPEED 25

#define RIGHT_SLOW_X 60
#define RIGHT_STOP_X 75

enum MainState { READY, SCANNING, ATTACK };
enum USIndex { US_FRONT = 0, US_RIGHT = 1, US_BACK = 2, US_LEFT = 3 };

MainState state = READY;

const uint8_t trigPins[US_COUNT] = { TRIG_F, TRIG_R, TRIG_B, TRIG_L };
const uint8_t echoPins[US_COUNT] = { ECHO_F, ECHO_R, ECHO_B, ECHO_L };

volatile uint32_t echo_start[US_COUNT] = {0};
volatile uint32_t echo_duration[US_COUNT] = {0};
volatile bool echo_done[US_COUNT] = {false};

float us_dist_cm[US_COUNT] = {
  US_INVALID_DISTANCE, US_INVALID_DISTANCE, US_INVALID_DISTANCE, US_INVALID_DISTANCE
};
uint8_t us_invalid_count[US_COUNT] = {0};
uint8_t current_us = US_COUNT - 1;
uint32_t last_trigger_time = 0;
uint32_t last_display_time = 0;

bool btnPressed(int pin){
  static unsigned long last[40] = {0};
  if(digitalRead(pin) == LOW && millis() - last[pin] > 200){
    last[pin] = millis();
    return true;
  }
  return false;
}
// ------------------ Ultrasonic ------------------
bool isValidUS(float dist_cm){
  return dist_cm < US_INVALID_DISTANCE;
}

void echoISR(uint8_t i){
  if(digitalRead(echoPins[i]) == HIGH){
    echo_start[i] = micros();
  }
  else{
    echo_duration[i] = micros() - echo_start[i];
    echo_done[i] = true;
  }
}

void echoFrontISR(){ echoISR(US_FRONT); }
void echoRightISR(){ echoISR(US_RIGHT); }
void echoBackISR(){ echoISR(US_BACK); }
void echoLeftISR(){ echoISR(US_LEFT); }

void triggerUS(uint8_t i){
  digitalWrite(trigPins[i], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[i], LOW);
}

void updateFilteredUS(uint8_t i, float raw_dist_cm){
  us_invalid_count[i] = 0;
  if(!isValidUS(us_dist_cm[i])){
    us_dist_cm[i] = raw_dist_cm;
  }
  else{
    us_dist_cm[i] = us_dist_cm[i] * (1.0f - US_FILTER_ALPHA) + raw_dist_cm * US_FILTER_ALPHA;
  }
}

void markInvalidUS(uint8_t i){
  if(us_invalid_count[i] < US_INVALID_LIMIT){
    us_invalid_count[i]++;
  }
  if(us_invalid_count[i] >= US_INVALID_LIMIT){
    us_dist_cm[i] = US_INVALID_DISTANCE;
  }
}

void updateUS(){
  if(millis() - last_trigger_time >= 50){
    last_trigger_time = millis();
    current_us++;
    if(current_us >= US_COUNT){
      current_us = 0;
    }

    echo_done[current_us] = false;
    triggerUS(current_us);
  }

  for(uint8_t i = 0; i < US_COUNT; i++){
    if(echo_done[i]){
      noInterrupts();
      uint32_t duration = echo_duration[i];
      echo_done[i] = false;
      interrupts();

      if(duration > 100 && duration < 12000){
        updateFilteredUS(i, duration * 0.0343f / 2.0f);
      }
      else{
        markInvalidUS(i);
      }
    }
  }
}

void setupUS(){
  for(uint8_t i = 0; i < US_COUNT; i++){
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }

  attachInterrupt(digitalPinToInterrupt(ECHO_F), echoFrontISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_R), echoRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_B), echoBackISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_L), echoLeftISR, CHANGE);
}

void applySideUSBrake(int16_t &vx){
  float left = us_dist_cm[US_LEFT];
  float right = us_dist_cm[US_RIGHT];
  if(!isValidUS(left)){
    return;
  }
  if(vx >= 0){
    return; 
  }

  if(left <= SIDE_STOP_DIST){
    vx = 0;
    return;
  }

  if(left < SIDE_SLOW_DIST){
    float t = (SIDE_SLOW_DIST - left) / (SIDE_SLOW_DIST - SIDE_STOP_DIST);
    t = constrain(t, 0.0f, 1.0f);

    float scale = exp(-SIDE_SLOW_EXP * t);
    vx = (int16_t)round(vx * scale);
  }

  if(isValidUS(right)){
    if(right <= SIDE_DANGER_DIST){
      vx = -SIDE_PUSH_SPEED;  // 往左
      return;
    }

    if(vx > 0){
      if(right <= SIDE_STOP_DIST){
        vx = 0;
      }
      else if(right < SIDE_SLOW_DIST){
        float t = (SIDE_SLOW_DIST - right) / (SIDE_SLOW_DIST - SIDE_STOP_DIST);
        t = constrain(t, 0.0f, 1.0f);

        float scale = exp(-SIDE_SLOW_EXP * t);
        vx = (int16_t)round(vx * scale);
      }
    }
  }
}
// ------------------ Omni Mirror ------------------
void applyOmniEdgeBrake(int16_t &vx){
  if(!maixPosData.valid || maixPosData.status != 2){
    return;
  }

  float x = maixPosData.x;

  if(vx <= 0){
    return; // 沒有往右，不限制
  }

  if(x >= RIGHT_STOP_X){
    vx = 0;
    return;
  }

  if(x >= RIGHT_SLOW_X){
    float t = (x - RIGHT_SLOW_X) / (RIGHT_STOP_X - RIGHT_SLOW_X);
    t = constrain(t, 0.0f, 1.0f);

    float scale = exp(-SIDE_SLOW_EXP * t);
    vx = (int16_t)round(vx * scale);
  }
}

// ------------------ Communication ------------------
void sendMovePacket(int16_t vx, int16_t vy){
  uint8_t packet[8];
  packet[0] = 0xAA;
  packet[1] = 0xAA;
  packet[2] = vx & 0xFF;
  packet[3] = (vx >> 8) & 0xFF;
  packet[4] = vy & 0xFF;
  packet[5] = (vy >> 8) & 0xFF;
  packet[6] = packet[2] + packet[3] + packet[4] + packet[5];
  packet[7] = 0xEE;
  Serial8.write(packet, 8);
}

void printUSValue(float dist_cm){
  if(!isValidUS(dist_cm)){
    display.print("---");
  }
  else{
    display.print((int)round(dist_cm));
  }
}

// ------------------ Display ------------------
void drawRunScreen(const char* mode, int16_t vx, int16_t vy){
  if(millis() - last_display_time < 100){
    return;
  }
  last_display_time = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 22);
  display.print("X:");
  printUSValue(maixPosData.x);
  display.print("Y:");
  printUSValue(maixPosData.y);

  display.setCursor(0, 34);
  if(ballData.valid){
    display.print("A:");
    display.print(ballData.angle);
  }
  else{
    display.print(mode);
  }

  display.setCursor(0, 46);
  display.print("Vx:");
  display.print(vx);

  display.setCursor(0, 56);
  display.print("Vy:");
  display.print(vy);
  //display.print(" L:");
  //printUSValue(us_dist_cm[US_LEFT]);
  //display.print(" R:");
  //printUSValue(us_dist_cm[US_RIGHT]);

  int cx = 100;
  int cy = 43;
  int r = 18;
  display.drawCircle(cx, cy, r, SSD1306_WHITE);

  if(ballData.valid){
    float rad = (ballData.angle + 180) * DtoR_const;
    int x = cx + cos(rad) * r;
    int y = cy - sin(rad) * r;
    display.drawLine(cx, cy, x, y, SSD1306_WHITE);
    display.fillCircle(x, y, 2, SSD1306_WHITE);
  }
  else{
    display.drawLine(cx - 5, cy - 5, cx + 5, cy + 5, SSD1306_WHITE);
    display.drawLine(cx + 5, cy - 5, cx - 5, cy + 5, SSD1306_WHITE);
  }

  display.display();
}

void drawState(const char* text){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 22);
  display.println(text);
  display.display();
}

// ------------------ Motion ------------------
void chaseBall(int16_t &vx, int16_t &vy){
  float moving_degree = ballData.angle;
  float ballspeed = 30;

  if(ballData.dist > 15){
    moving_degree = ballData.angle;
    if(ballData.angle >= 80 && ballData.angle <= 100){
      moving_degree = 90;
    }
  }
  else{
    if(ballData.angle >= 80 && ballData.angle <= 100){
      moving_degree = 90;
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

  vx = (int16_t)round(ballspeed * cos(moving_degree * DtoR_const));
  vy = (int16_t)round(ballspeed * sin(moving_degree * DtoR_const));
}

void setup(){
  Robot_Init();
  setupUS();
}

void loop(){
  updateUS();

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

      drawState("SAVED");
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
  readMaixPosition();
  readBNO085Yaw();
  ballsensor();

  int16_t vx = 0;
  int16_t vy = 0;

  if(ballData.valid){
    chaseBall(vx, vy);
  }
  //applySideUSBrake(vx);
  //applyOmniEdgeBrake(vx);

  ballData.Vx = vx;
  ballData.Vy = vy;
  sendMovePacket(vx, vy);
  drawRunScreen(ballData.valid ? "BALL" : "HOME", vx, vy);
  //Serial.print("Left US: ");
  //Serial.println(us_dist_cm[US_LEFT]);  
}
