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

  state = READY;

  Serial8.write(CMD_STOP);
  drawState("READY");
}

void loop() {
  
    int16_t vx = 0;
    int16_t vy = 0;
    int8_t aim_offset = 0;
    // 只讀 Maix 相機
    readMaix();
    bool eat_ball = digitalRead(A16) == LOW;
    if (eat_ball) {
      //resetBallField();

      vx = 0;
      vy = 0;
      aim_offset
      //kicker_control(true);
    }
    sendMovePacket(vx, vy, -7);

    //delay(10);
  
}