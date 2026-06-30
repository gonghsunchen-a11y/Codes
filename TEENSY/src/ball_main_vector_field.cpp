#include <Arduino.h>
#include <Robot.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>

Servo ESC;

#define CMD_ATTACK 0xAA
#define CMD_LINECAL_START 0xCC
#define CMD_LINECAL_SAVE 0xEE
#define CMD_LINECAL_DONE 0xDD

#define RIGHT_SLOW_X 55.0f
#define RIGHT_STOP_X 70.0f
#define LEFT_SLOW_X -55.0f
#define LEFT_STOP_X -70.0f
#define FRONT_SLOW_Y 70.0f
#define FRONT_STOP_Y 90.0f
#define BACK_SLOW_Y -70.0f
#define BACK_STOP_Y -90.0f

#define EAT_BALL_IR_PIN A16
#define EAT_BALL_WINDOW 20
#define EAT_BALL_LOW_THRESHOLD 5

enum MainState { READY, SCANNING, ATTACK };
MainState state = READY;

bool eat_ball = false;

struct BallFieldState {
  int8_t orbit_side = 0;
  bool capture_mode = false;
  bool angle_initialized = false;
  float last_angle = 90.0f;
  float angle_rate = 0.0f;
  float predicted_error = 0.0f;
  uint32_t last_update_us = 0;
} ballField;

float normalizeAngle180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

bool btnPressed(int pin) {
  static uint32_t last[40] = {0};

  if (digitalRead(pin) == LOW && millis() - last[pin] > 200) {
    last[pin] = millis();
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

/*bool readEatBall() {
  static uint8_t hits[EAT_BALL_WINDOW] = {0};
  static uint8_t index = 0;
  static uint8_t filled = 0;

  int value = analogRead(EAT_BALL_IR_PIN);

  hits[index] = value < EAT_BALL_LOW_THRESHOLD;
  index = (index + 1) % EAT_BALL_WINDOW;

  if (filled < EAT_BALL_WINDOW) {
    filled++;
  }

  for (uint8_t i = 0; i < filled; i++) {
    if (hits[i]) return true;
  }

  return false;
}
*/
void resetBallField() {
  ballField.orbit_side = 0;
  ballField.capture_mode = false;
  ballField.angle_initialized = false;
  ballField.angle_rate = 0.0f;
  ballField.predicted_error = 0.0f;
}

void updateBallPrediction(float ball_angle) {
  uint32_t now = micros();

  if (!ballField.angle_initialized) {
    ballField.angle_initialized = true;
    ballField.last_angle = ball_angle;
    ballField.last_update_us = now;
    ballField.angle_rate = 0.0f;
  } else {
    float dt = (now - ballField.last_update_us) * 0.000001f;
    dt = constrain(dt, 0.005f, 0.1f);

    float delta = normalizeAngle180(ball_angle - ballField.last_angle);
    float measured_rate = delta / dt;
    ballField.angle_rate =
        0.75f * ballField.angle_rate + 0.25f * measured_rate;

    ballField.last_angle = ball_angle;
    ballField.last_update_us = now;
  }

  const float look_ahead_seconds = 0.10f;
  float predicted_angle =
      ball_angle + ballField.angle_rate * look_ahead_seconds;
  ballField.predicted_error =
      normalizeAngle180(predicted_angle - 90.0f);

  if (!ballField.capture_mode &&
      fabs(ballField.predicted_error) < 18.0f) {
    ballField.capture_mode = true;
  } else if (ballField.capture_mode &&
             fabs(ballField.predicted_error) > 32.0f) {
    ballField.capture_mode = false;
  }
}

void applyCaptureControl(int16_t &vx, int16_t &vy) {
  const float speed = 60.0f;
  const float kp = 0.9f;
  const float kd = 0.06f;
  const float max_lateral = 35.0f;

  float lateral =
      -(kp * ballField.predicted_error + kd * ballField.angle_rate);
  lateral = constrain(lateral, -max_lateral, max_lateral);

  float forward_squared = speed * speed - lateral * lateral;
  float forward = sqrtf(fmaxf(0.0f, forward_squared));

  vx = (int16_t)roundf(lateral);
  vy = (int16_t)roundf(forward);
}

void applyBallVectorField(int16_t &vx, int16_t &vy) {
  float angle = maixPosData.ball_angle;
  float distance = maixPosData.ball_dist;
  float radians = angle * DtoR_const;

  if (ballField.orbit_side == 0) {
    ballField.orbit_side =
        (angle > 90.0f && angle <= 270.0f) ? 1 : -1;
  }

  float radial_x = cosf(radians);
  float radial_y = sinf(radians);
  float tangent_x = -ballField.orbit_side * radial_y;
  float tangent_y = ballField.orbit_side * radial_x;

  const float target_distance = 60.0f;
  float radial_speed = (distance - target_distance) * 2.2f;
  radial_speed = constrain(radial_speed, -15.0f, 55.0f);

  float near_ratio = (75.0f - distance) / 20.0f;
  near_ratio = constrain(near_ratio, 0.0f, 1.0f);

  float tangent_speed = 20.0f + 50.0f * near_ratio;

  float front_blend =
      1.0f - fabs(ballField.predicted_error) / 30.0f;
  front_blend = constrain(front_blend, 0.0f, 1.0f);

  tangent_speed *= 1.0f - front_blend;
  radial_speed =
      radial_speed * (1.0f - front_blend) + 60.0f * front_blend;

  float command_x =
      radial_speed * radial_x + tangent_speed * tangent_x;
  float command_y =
      radial_speed * radial_y + tangent_speed * tangent_y;

  const float max_speed = 70.0f;
  float magnitude = sqrtf(command_x * command_x + command_y * command_y);

  if (magnitude > max_speed) {
    float scale = max_speed / magnitude;
    command_x *= scale;
    command_y *= scale;
  }

  vx = (int16_t)roundf(command_x);
  vy = (int16_t)roundf(command_y);
}

void applyIRChase(int16_t &vx, int16_t &vy) {
  float radians = ballData.angle * DtoR_const;
  float speed = constrain(map(ballData.dist,6,2,40,70),40.0f,70.0f);

  vx = (int16_t)roundf(speed * cosf(radians));
  vy = (int16_t)roundf(speed * sinf(radians));
}

void applyBoundaryVectorField(int16_t &vx, int16_t &vy) {
  if (!maixPosData.valid) return;

  float x = maixPosData.x;
  float y = maixPosData.y;
  float out_x = vx;
  float out_y = vy;
  const float push = 35.0f;

  if (x > RIGHT_SLOW_X) {
    float t = (x - RIGHT_SLOW_X) / (RIGHT_STOP_X - RIGHT_SLOW_X);
    t = constrain(t, 0.0f, 1.0f);
    if (out_x > 0.0f) out_x *= 1.0f - t;
    out_x -= push * t * t;
  }

  if (x < LEFT_SLOW_X) {
    float t = (LEFT_SLOW_X - x) / (LEFT_SLOW_X - LEFT_STOP_X);
    t = constrain(t, 0.0f, 1.0f);
    if (out_x < 0.0f) out_x *= 1.0f - t;
    out_x += push * t * t;
  }

  if (y > FRONT_SLOW_Y) {
    float t = (y - FRONT_SLOW_Y) / (FRONT_STOP_Y - FRONT_SLOW_Y);
    t = constrain(t, 0.0f, 1.0f);
    if (out_y > 0.0f) out_y *= 1.0f - t;
    out_y -= push * t * t;
  }

  if (y < BACK_SLOW_Y) {
    float t = (BACK_SLOW_Y - y) / (BACK_SLOW_Y - BACK_STOP_Y);
    t = constrain(t, 0.0f, 1.0f);
    if (out_y < 0.0f) out_y *= 1.0f - t;
    out_y += push * t * t;
  }

  vx = (int16_t)constrain(roundf(out_x), -100.0f, 100.0f);
  vy = (int16_t)constrain(roundf(out_y), -100.0f, 100.0f);
}

void sendMovePacket(int16_t vx, int16_t vy, int8_t aim_offset) {
  vx = constrain(vx, -100, 100);
  vy = constrain(vy, -100, 100);

  uint8_t packet[7];
  packet[0] = 0xAA;
  packet[1] = 0xAA;
  packet[2] = (uint8_t)(int8_t)vx;
  packet[3] = (uint8_t)(int8_t)vy;
  packet[4] = (uint8_t)aim_offset;
  packet[5] = packet[2] + packet[3] + packet[4];
  packet[6] = 0xEE;

  Serial8.write(packet, sizeof(packet));
}

void setup() {
  Robot_Init();
  pinMode(EAT_BALL_IR_PIN, INPUT);

  ESC.attach(A17, 1000, 2000);
  ESC.writeMicroseconds(1000);
  delay(2000);

  for (int signal = 1000; signal <= 1500; signal += 10) {
    ESC.writeMicroseconds(signal);
    delay(20);
  }

  ESC.writeMicroseconds(1500);
  delay(5000);
}

void loop() {
  

  if (state == READY) {
    if (btnPressed(BTN_ENTER)) {
      state = SCANNING;
      Serial8.write(CMD_LINECAL_START);
      drawState("SCANNING");
      return;
    }

    if (btnPressed(BTN_UP)) {
      state = ATTACK;
      Serial8.write(CMD_ATTACK);
      drawState("ATTACK");
      delay(100);
      return;
    }

    drawState("READY");
    return;
  }

  if (state == SCANNING) {
    if (btnPressed(BTN_ESC)) {
      Serial8.write(CMD_LINECAL_SAVE);
      state = READY;

      uint32_t start = millis();
      while (millis() - start < 2000) {
        if (Serial8.available() &&
            Serial8.read() == CMD_LINECAL_DONE) {
          break;
        }
      }

      drawState("SAVED\nREADY");
      delay(500);
      return;
    }

    drawState("SCANNING");
    return;
  }

  readBNO085Yaw();
  ballsensor();
  readMaix();
  eat_ball = digitalRead(EAT_BALL_IR_PIN) == LOW;
  ESC.writeMicroseconds(1625);

  int16_t vx = 0;
  int16_t vy = 0;
  int8_t aim_offset = 0;
  if (eat_ball) {
    resetBallField();
    vx = 0;
    vy = 80;
    FrontCam();
    if (frontcam.valid) {
      aim_offset =(int8_t)constrain(frontcam.offset * 1.5f, -45.0f, 45.0f);
    }

    //kicker_control(true);
  }
  else {
    if (ballData.valid) {
      if (maixPosData.valid && maixPosData.ball_found) {
        updateBallPrediction(maixPosData.ball_angle);

        if (ballField.capture_mode) {
          applyCaptureControl(vx, vy);
        } else {
          applyBallVectorField(vx, vy);
        }
      } else {
        resetBallField();
        applyIRChase(vx, vy);
      }
    } else {
      resetBallField();
    }
      //kicker_control(false);
  }

  applyBoundaryVectorField(vx, vy);
  sendMovePacket(vx, vy, aim_offset);
}
