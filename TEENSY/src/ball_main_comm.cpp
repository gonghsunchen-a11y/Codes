#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>
#include <Servo.h>
Servo ESC;  // Create ESC control object


#define CMD_ATTACK 0xAA
#define CMD_LINECAL_START 0xCC
#define CMD_LINECAL_SAVE 0xEE
#define CMD_LINECAL_DONE 0xDD

#define RIGHT_SLOW_X 55
#define RIGHT_STOP_X 70
#define LEFT_SLOW_X -55
#define LEFT_STOP_X -70
#define FRONT_SLOW_Y 70
#define FRONT_STOP_Y 90
#define BACK_SLOW_Y -70
#define BACK_STOP_Y -90
#define SIDE_SLOW_EXP 0.05f

#define EAT_BALL_IR_PIN A16
#define EAT_BALL_WINDOW 20
#define EAT_BALL_LOW_THRESHOLD 5

#define TX4 A3
#define RX4 A2
#define TX3 A1
#define RX3 A0

bool eat_ball = false;

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
/*
extern "C" void startup_middle_hook(void);

FLASHMEM void startup_middle_hook(void){
  pinMode(TX4,INPUT);
  pinMode(RX4,INPUT);
  pinMode(TX3,INPUT);
  pinMode(RX3,INPUT);
}
*/
void setup(){
  //pinMode(TX4,INPUT);
  //pinMode(RX4,INPUT);
  //pinMode(TX3,INPUT);
  //pinMode(RX3,INPUT);
  //delay(5000);  
  Robot_Init();
  pinMode(EAT_BALL_IR_PIN, INPUT);
  ESC.attach(A17,1000,2000);

    // Start at minimum throttle
    Serial.println("Initializing ESC...");
    ESC.writeMicroseconds(1000);
    delay(2000);  // Give the ESC time to detect the signal
    Serial.println("Started at 1000 µs");

    // Slowly ramp from 1000 to 1500 for safe arming
    for (int signal = 1000; signal <= 1500; signal += 10) {
        ESC.writeMicroseconds(signal);
        Serial.println(signal);
        delay(20);  // Slow enough for ESC to recognize change
    }
    ESC.writeMicroseconds(1500);
    Serial.println("Initialization complete.");
    delay(5000);  // Optional pause before starting loop
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
void applyOmniEdgeBrake(int16_t &vx,int16_t &vy){
  //Serial.print("x");Serial.print(maixPosData.x);
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
  if(vy>0){  if(x >= FRONT_STOP_Y){
      vy = 0;
      return;
    }

    if(x >= FRONT_SLOW_Y){
      float t = (x -FRONT_SLOW_Y) / (FRONT_STOP_Y - FRONT_SLOW_Y);
      t = constrain(t, 0.0f, 1.0f);

      float scale = exp(-SIDE_SLOW_EXP * t);
      vx = (int16_t)round(vx * scale);
    }

    return;
  }
  if(vy<0){  if(x >= BACK_STOP_Y){
      vy = 0;
      return;
    }

    if(x >= BACK_SLOW_Y){
      float t = (x -BACK_SLOW_Y) / (BACK_STOP_Y - BACK_SLOW_Y);
      t = constrain(t, 0.0f, 1.0f);

      float scale = exp(-SIDE_SLOW_EXP * t);
      vx = (int16_t)round(vx * scale);
    }

    return;
  }
}



bool readEatBall(){
  static uint8_t hits[EAT_BALL_WINDOW] = {0};
  static uint8_t idx = 0;
  static uint8_t filled = 0;

  int value = analogRead(EAT_BALL_IR_PIN);

  hits[idx] = (value < EAT_BALL_LOW_THRESHOLD) ? 1 : 0;
  idx = (idx + 1) % EAT_BALL_WINDOW;

  if(filled < EAT_BALL_WINDOW){
    filled++;
  }

  bool eat_state = false;
  for(uint8_t i = 0; i < filled; i++){
    if(hits[i]){
      eat_state = true;
      break;
    }
  }
/*
  Serial.print("value=");
  Serial.print(value);
  Serial.print(" hit=");
  Serial.print(value < EAT_BALL_LOW_THRESHOLD);
  Serial.print(" eat=");
  Serial.println(eat_state);
*/
  return eat_state;
}

void loop(){
  

  if(state == READY){
  if(btnPressed(BTN_ENTER)){
    state = SCANNING;
    Serial8.write(CMD_LINECAL_START);
    drawState("SCANNING");
    return;
  }

  if(btnPressed(BTN_UP)){
    state = ATTACK;
    Serial8.write(CMD_ATTACK);
    drawState("ATTACK");
    delay(100);   // 給副控時間進 ATTACK
    return;
  }

  drawState("READY");
  return;
}

if(state == SCANNING){
  if(btnPressed(BTN_ESC)){
    Serial8.write(CMD_LINECAL_SAVE);
    state = READY;

    unsigned long t = millis();
    while(millis() - t < 2000){
      if(Serial8.available() && Serial8.read() == CMD_LINECAL_DONE) break;
    }

    drawState("SAVED\nREADY");
    delay(500);
    return;
  }

  drawState("SCANNING");
  return;
}
  //Serial.print("In");
  //showBallDirection();
  readBNO085Yaw();
  ballsensor();
  readMaix();
  
  FrontCam();
  //Serial.print(frontcam.offset);
  eat_ball = readEatBall();
  int16_t vx = 0;
  int16_t vy = 0;
  int8_t aim_offset = 0;
  ESC.writeMicroseconds(1625);
  if(maixPosData.valid ){Serial.println("yes");}
  if(frontcam.valid ){Serial.println("yesyes");}
  if(ballData.valid){
    //Serial.print(" dis");Serial.println(ballData.dist);
    //Serial.print(" angle");Serial.print(ballData.angle);

    if(maixPosData.valid && maixPosData.ball_found){

      float moving_degree = maixPosData.ball_angle;
      float offset = 0;
      float ballspeed = constrain(map(maixPosData.ball_dist, 55, 80, 40, 60), 40, 60);
      //float ballspeedVx = constrain(map(maixPosData.ball_dist, 70, 85, 30, 50), 30, 50);
      //float ballspeedVy = constrain(map(maixPosData.ball_dist, 70, 85, 25, 50), 25, 50);

      //Serial.println(maixPosData.ball_dist);
      if(maixPosData.ball_angle >= 83 && maixPosData.ball_angle <= 98){
        moving_degree = 90;
        ballspeed = 50;
        digitalWrite(LED_BUILTIN,HIGH);
        //kicker_control(1);
        //Serial.println(frontcam.offset);
        /*if(maixPosData.ball_dist <= 54 && maixPosData.ball_angle >=  87 && maixPosData.ball_angle <= 93){
          vx = 0;
          vy = 80;   // 90度往前衝
          if(frontcam.valid){
            Serial.print(frontcam.x);
            aim_offset = constrain(frontcam.offset, -45, 45);
          }
          else{aim_offset = 0;}
        }*/
      }
      else if(maixPosData.ball_angle > 100 && maixPosData.ball_angle < 155){
        float offsetRatio = exp(-0.1 * (maixPosData.ball_dist - 60));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 80 * offsetRatio;
        moving_degree = maixPosData.ball_angle + offset;
        digitalWrite(LED_BUILTIN,LOW);
        //float angleError = fabs(ballData.angle - 90);
        //float smoothWeight = constrain(angleError / 25.0f, 0.0f, 1.0f);
      }
      else if(maixPosData.ball_angle >= 155 && maixPosData.ball_angle <= 270){
        float offsetRatio = exp(-0.03 * (maixPosData.ball_dist - 60));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 100 * offsetRatio;
        //Serial.print(" offset=");Serial.print(offset);
        moving_degree = maixPosData.ball_angle + offset;
        digitalWrite(LED_BUILTIN,LOW);
      }
      else if(maixPosData.ball_angle < 80 && maixPosData.ball_angle >= 25){
        float offsetRatio = exp(-0.1 * (maixPosData.ball_dist - 60));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 80 * offsetRatio;
        moving_degree = maixPosData.ball_angle - offset;
        digitalWrite(LED_BUILTIN,LOW);
      }
      else if(maixPosData.ball_angle < 25 || maixPosData.ball_angle > 270){
        float offsetRatio = exp(-0.03 * (maixPosData.ball_dist - 60));
        offsetRatio = constrain(offsetRatio, 0.0, 1.0);
        offset = 100*offsetRatio;
        moving_degree = maixPosData.ball_angle - offset;
        digitalWrite(LED_BUILTIN,LOW);
      }
    
      if(moving_degree < 0) moving_degree += 360;
      if(moving_degree >= 360) moving_degree -= 360;
      vx = (int16_t)round(ballspeed * cos(moving_degree * DtoR_const));
      vy = (int16_t)round(ballspeed * sin(moving_degree * DtoR_const));

      Serial.print(" ball=");Serial.print(maixPosData.ball_angle);
      Serial.print(" balldist=");Serial.print(maixPosData.ball_dist);
      Serial.print(" balldist=");Serial.print(maixPosData.ball_dist);
      Serial.print(" move=");Serial.println(moving_degree);

    }
    else{
      if(ballData.valid){
      float moving_degree = ballData.angle;
      float ballspeed = constrain(map(ballData.dist, 8, 2, 40, 60), 40, 60);
      //if(ballData.dist>=6){ballspeed =50;}
      //else{ballspeed=80;}
      //Serial.print(" valid");Serial.print(ballData.valid);
      //Serial.print(" dis");Serial.println(ballData.dist);
      //Serial.print(" angle");Serial.print(ballData.angle);
      //Serial.print(" moving");Serial.println(moving_degree);
      if(moving_degree < 0) moving_degree += 360;
      if(moving_degree >= 360) moving_degree -= 360;
      vx = (int16_t)round(ballspeed * cos(moving_degree * DtoR_const));
      vy = (int16_t)round(ballspeed * sin(moving_degree * DtoR_const));
      }
    }
  }
  else{
    vx = 0;
    vy = 0;
  }

  

  
 
  //Serial.println(aim_offset);
  //if(vy<40)vy=40;
  float angleError = fabs(ballData.angle - 90);
  float vxWeight = constrain(angleError / 25.0f, 0.0f, 1.0f);
  //vx = (int)round(vx* vxWeight);
  if(digitalRead(EAT_BALL_IR_PIN) == 0){
          //kicker_control(1);
          //Serial.println("eat");
          vx = 0;
          
          if(frontcam.valid){
            //Serial.print(frontcam.x);
            aim_offset = constrain(frontcam.offset*1.5, -45, 45);
            vy = 80;   // 90度往前衝
            
          }
          else{
            aim_offset = 0;
            vy = 50;   // 90度往前衝
          }
          kicker_control(1);
        }
        else{
          aim_offset = 0;
        }
  applyOmniEdgeBrake(vx,vy);
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
  //Serial.print(" vx=");Serial.print(vx);
  //Serial.print(" vy=");Serial.print(vy);
  //Serial.print(" eat=");Serial.println(eat_ball);

  uint8_t packet[7];

  packet[0] = 0xAA;
  packet[1] = 0xAA;
  packet[2] = (uint8_t)vx;
  packet[3] = (uint8_t)vy;
  packet[4] = (uint8_t)aim_offset;
  packet[5] = packet[2] + packet[3] + packet[4];
  packet[6] = 0xEE;

  //Serial.println("SEND PACKET");
  Serial8.write(packet, 7);
/*
  for(int i = 0; i < 7; i++){
  //if(packet[i] < 0x10) Serial.print("0");
  Serial.print(packet[i], HEX);
  Serial.print(" ");
  }
  Serial.println();*/
}
