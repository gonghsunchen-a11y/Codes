#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define CMD_ATTACK 0xAA
#define CMD_STOP   0xBB

enum MainState {
  READY,
  ATTACK
};

MainState state = READY;

bool btnPressed(int pin) {
  static uint32_t last_time[40] = {0};

  if (digitalRead(pin) == LOW &&
      millis() - last_time[pin] > 200) {

    last_time[pin] = millis();
    return true;
  }

  return false;
}

void drawState(const char *text) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println(text);
  display.display();
}

void sendMovePacket(
    int16_t vx,
    int16_t vy,
    int8_t aim_offset) {

  vx = constrain(vx, -100, 100);
  vy = constrain(vy, -100, 100);

  uint8_t packet[7];

  packet[0] = 0xAA;
  packet[1] = 0xAA;
  packet[2] = (uint8_t)(int8_t)vx;
  packet[3] = (uint8_t)(int8_t)vy;
  packet[4] = (uint8_t)aim_offset;

  packet[5] =
      packet[2] +
      packet[3] +
      packet[4];

  packet[6] = 0xEE;

  Serial8.write(
      packet,
      sizeof(packet)
  );
}

// 使用 Maix 相機計算繞球 vx、vy
void calculateCameraOrbit(
    int16_t &vx,
    int16_t &vy) {

  vx = 0;
  vy = 0;

  if (!maixPosData.valid ||
      !maixPosData.ball_found) {
    return;
  }

  float ball_angle =
      maixPosData.ball_angle;

  float ball_distance =
      maixPosData.ball_dist;

  // 90° 代表球在車頭正前方
  float radians =
      ball_angle * DtoR_const;

  float ball_x =
      cosf(radians);

  float ball_y =
      sinf(radians);

  const float TARGET_DISTANCE = 55.0f;
  const float ORBIT_SPEED = 30.0f;

  // 正值：球太遠，要靠近
  // 負值：球太近，要後退
  float radial_speed =
      (ball_distance -
       TARGET_DISTANCE) * 0.3f;

  radial_speed = constrain(
      radial_speed,
      -12.0f,
      15.0f
  );

  // 向球靠近＋沿切線繞球
  float command_x =
      radial_speed * ball_x +
      ORBIT_SPEED * ball_y;

  float command_y =
      radial_speed * ball_y -
      ORBIT_SPEED * ball_x;

  vx =
      (int16_t)roundf(command_x);

  vy =
      (int16_t)roundf(command_y);
}

void setup() {
  Robot_Init();
  pinMode(A16,INPUT);
  state = READY;

  Serial8.write(CMD_STOP);
  drawState("READY");
}

void loop() {
  
    int16_t vx = 0;
    int16_t vy = 0;
    int8_t aim_offset = 0;
    static uint32_t eat_ball_start = 0;
    static bool was_eating_ball = false;
    static bool kick_sent = false;
    static int x = 60;
    // 只讀 Maix 相機
    readMaix();
    ballsensor();
    bool eat_ball = digitalRead(A16) == LOW;
    if (eat_ball) {
  if (!was_eating_ball) {
    eat_ball_start = millis();
    kick_sent = false;
  }

  was_eating_ball = true;

  vx = 0;
  vy = 50;
  //aim_offset = -15;
  if(x < -50){
  aim_offset = -15;}
  else if(x > 50){ aim_offset = 15;}
  else{aim_offset = 0;}

  if (!kick_sent &&
      millis() - eat_ball_start >= 75UL) {
        Serial.print("kick:");
    kicker_control(true);
    kick_sent = true;
  } else {
    kicker_control(false);
  }
} else {
  Serial.println(maixPosData.ball_angle);
  was_eating_ball = false;
  eat_ball_start = 0;
  kick_sent = false;

  kicker_control(false);
  if(ballData.valid){
      
      //Serial.print(" dis");Serial.println(ballData.dist);
      //Serial.print(" angle");Serial.print(ballData.angle);

      if(maixPosData.valid && maixPosData.ball_found){

        float moving_degree = maixPosData.ball_angle;
        float offset = 0;
        float ballspeed = constrain(map(maixPosData.ball_dist, 55, 80, 30, 70), 30, 70);
        //float ballspeedVx = constrain(map(maixPosData.ball_dist, 70, 85, 30, 50), 30, 50);
        //float ballspeedVy = constrain(map(maixPosData.ball_dist, 70, 85, 25, 50), 25, 50);

        //Serial.println(maixPosData.ball_dist);
        if(maixPosData.ball_angle >= 83 && maixPosData.ball_angle <= 97){
          //moving_degree = maixPosData.ball_angle;
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
        }else if(maixPosData.ball_angle > 97 && maixPosData.ball_angle <= 130){
          ballspeed = constrain(map(maixPosData.ball_dist, 55, 80, 30, 50),30,50);
          float offsetRatio = exp(-0.1 * (maixPosData.ball_dist - 65));
          offsetRatio = constrain(offsetRatio, 0.0, 1.0);
          offset = 20 * offsetRatio;
          moving_degree = maixPosData.ball_angle + offset;
        }
        else if(maixPosData.ball_angle > 130 && maixPosData.ball_angle < 155){
          float offsetRatio = exp(-0.1 * (maixPosData.ball_dist - 60));
          offsetRatio = constrain(offsetRatio, 0.0, 1.0);
          offset = 90 * offsetRatio;
          moving_degree = maixPosData.ball_angle + offset;
          digitalWrite(LED_BUILTIN,LOW);
          //float angleError = fabs(ballData.angle - 90);
          //float smoothWeight = constrain(angleError / 25.0f, 0.0f, 1.0f);
        }
        else if(maixPosData.ball_angle >= 155 && maixPosData.ball_angle <= 270){
          float offsetRatio = exp(-0.03 * (maixPosData.ball_dist - 65));
          offsetRatio = constrain(offsetRatio, 0.0, 1.0);
          offset = 100 * offsetRatio;
          //Serial.print(" offset=");Serial.print(offset);
          moving_degree = maixPosData.ball_angle + offset;
          digitalWrite(LED_BUILTIN,LOW);
        }
        else if(maixPosData.ball_angle >= 50 && maixPosData.ball_angle < 83){
          ballspeed =constrain(map(maixPosData.ball_dist, 55, 80, 30, 50),30,50); //30;
          float offsetRatio = exp(-0.1 * (maixPosData.ball_dist - 65));
          offsetRatio = constrain(offsetRatio, 0.0, 1.0);
          offset = 20 * offsetRatio;
          moving_degree = maixPosData.ball_angle - offset;
        }
        else if(maixPosData.ball_angle < 50 && maixPosData.ball_angle >=25){
          float offsetRatio = exp(-0.1 * (maixPosData.ball_dist - 60));
          offsetRatio = constrain(offsetRatio, 0.0, 1.0);
          offset = 90 * offsetRatio;
          moving_degree = maixPosData.ball_angle - offset;
          digitalWrite(LED_BUILTIN,LOW);
        }
        else if(maixPosData.ball_angle < 25 || maixPosData.ball_angle > 270){
          float offsetRatio = exp(-0.03 * (maixPosData.ball_dist - 60));
          offsetRatio = constrain(offsetRatio, 0.0, 1.0);
          offset = 100 * offsetRatio;
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
        Serial.print(" ballspeed=");Serial.println(ballspeed);

      }
      else{
        if(ballData.valid){
        float moving_degree = ballData.angle;
        float ballspeed = constrain(map(ballData.dist, 5, 2, 30, 50), 30, 50);
        //if(ballData.dist>=6){ballspeed =50;}
        //else{ballspeed=80;}
        //Serial.print(" valid");Serial.print(ballData.valid);
        Serial.print(" dis");Serial.println(ballData.dist);
        Serial.print(" angle");Serial.print(ballData.angle);
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
}float angleError = fabs(ballData.angle - 90);
  float vxWeight = constrain(angleError / 25.0f, 0.4f, 1.0f);
  vx = (int)round(vx* vxWeight);

    sendMovePacket(vx, vy, aim_offset);

    //delay(10);
  
}