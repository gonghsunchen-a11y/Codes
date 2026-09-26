#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define MAX_SPEED 50.0f
#define SLOW_DISTANCE 90.0f
#define STOP_DISTANCE 50.0f

int8_t arc_side = 1;

void setup() {
  Robot_Init();

  delay(1000);
  Serial.println("Goal receiver start");
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

void loop() {
  readGoal();

  int16_t vx = 0;
  int16_t vy = 0;

  if (goalData.valid||maixPosData.valid) {
    float speed;

    if (goalData.dist >= SLOW_DISTANCE) {
      speed = MAX_SPEED;
    }
    else if (goalData.dist <= STOP_DISTANCE) {
      speed = 0;
    }
    else {
      // 距離 90 -> 50，速度 50 -> 0
      speed = MAX_SPEED *
              (goalData.dist - STOP_DISTANCE) /
              (SLOW_DISTANCE - STOP_DISTANCE);
    }

    // 以正後方 270 度判斷位於球門哪一側
    float rear_error = goalData.angle - 270.0f;

    while (rear_error > 180.0f) rear_error -= 360.0f;
    while (rear_error < -180.0f) rear_error += 360.0f;

    // 加死區，避免在 270 度附近左右切換
    if (rear_error > 5.0f) {
      arc_side = 1;
    }
    else if (rear_error < -5.0f) {
      arc_side = -1;
    }

    float goal_radians = goalData.angle * PI / 180.0f;

    // 指向球門的單位向量
    float goal_x = cosf(goal_radians);
    float goal_y = sinf(goal_radians);

    // 球門圓周的切線方向
    float tangent_x = -arc_side * goal_y;
    float tangent_y =  arc_side * goal_x;

    // 25% 朝球門靠近，75% 沿彩虹半圓移動
    const float GOAL_PULL = 0.65f;

    float move_x =
        tangent_x * (1.0f - GOAL_PULL) +
        goal_x * GOAL_PULL;

    float move_y =
        tangent_y * (1.0f - GOAL_PULL) +
        goal_y * GOAL_PULL;

    // 正規化，保持原本 speed
    float magnitude = sqrtf(
        move_x * move_x +
        move_y * move_y
    );

    if (magnitude > 0.001f) {
      move_x /= magnitude;
      move_y /= magnitude;
    }

    vx = (int16_t)roundf(speed * move_x);
    vy = (int16_t)roundf(speed * move_y);

    Serial.print("angle=");
    Serial.print(goalData.angle);
    Serial.print(" dist=");
    Serial.print(goalData.dist);
    Serial.print(" ban=");
    Serial.print(maixPosData.ball_angle);
    Serial.print(" bdist=");
    Serial.print(maixPosData.ball_dist);
  }
  else {
    Serial.println("Goal not found");
  }
sendMovePacket(vx,vy,0);
}