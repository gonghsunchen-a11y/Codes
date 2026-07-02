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
#define CMD_STOP 0xBB

#define COM1 36
#define COM2 37

#define RIGHT_SLOW_X 55.0f
#define RIGHT_STOP_X 70.0f
#define LEFT_SLOW_X -55.0f
#define LEFT_STOP_X -70.0f
#define FRONT_SLOW_Y 50.0f
#define FRONT_STOP_Y 80.0f
#define BACK_SLOW_Y -50.0f
#define BACK_STOP_Y -80.0f
#define SIDE_SLOW_EXP 1.2f
#define SIDE_SLOW_DIST 60.0f
#define SIDE_STOP_DIST 30.0f
#define SIDE_DANGER_DIST 20.0f
#define SIDE_SLOW_EXP 2.0f
#define SIDE_PUSH_SPEED 25

#define EAT_BALL_IR_PIN A16
#define EAT_BALL_WINDOW 20
#define EAT_BALL_LOW_THRESHOLD 5

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
#define SIDE_SLOW_DIST 70.0f
#define SIDE_STOP_DIST 40.0f

float filtered_pos_x = 0.0f;
float filtered_pos_y = 0.0f;
bool filtered_pos_valid = false;


enum MainState { READY, SCANNING, ATTACK };
MainState state = READY;

bool eat_ball = false;
bool com_attack_active = false;
uint32_t last_stop_command_ms = 0;

enum CornerShotState {
  CORNER_IDLE,
  CORNER_BACKUP,
  CORNER_AIM,
  CORNER_SHOOT,
  CORNER_WAIT_BALL_LEAVE
};

CornerShotState cornerShotState = CORNER_IDLE;
uint32_t cornerShotTimer = 0;

struct BallFieldState {
  int8_t orbit_side = 0;
  bool capture_mode = false;
  bool angle_initialized = false;
  float last_angle = 90.0f;
  float angle_rate = 0.0f;
  float predicted_error = 0.0f;
  uint32_t last_update_us = 0;
} ballField;

enum USIndex { US_FRONT = 0, US_RIGHT = 1, US_BACK = 2, US_LEFT = 3 };
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
//------------------ Ultrasonic ------------------
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

      if(duration > 100 && duration < 15000){
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
void applySideUS(int16_t &vx){
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
      vx = 0;  // 往左
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
void applySideUSBrake(int16_t &vy){
  float front = us_dist_cm[US_FRONT];
  float back = us_dist_cm[US_BACK];
   if(front <= SIDE_DANGER_DIST){
      vy = -50;  // 往左
      return;
    }
  if(vy > 0){
      
    if(!isValidUS(front)){
      return;
    }
    
    /*if(maixPosData.y <=20){
      return; 
    }*/
    if(front <= SIDE_STOP_DIST){
      vy = 0;
      return;
    }

    if(front < SIDE_SLOW_DIST){
      float t = (SIDE_SLOW_DIST - front) / (SIDE_SLOW_DIST - SIDE_STOP_DIST);
      t = constrain(t, 0.0f, 1.0f);

      float scale = exp(-0.5 * t);
      vy = (int16_t)round(vy * scale);
    }
  }
  if(vy < 0){
    if(!isValidUS(back)){
      return;
    }
    
    if(maixPosData.y >= -20){
      return; 
    }
    if(back <= 30){
      vy = 0;
      return;
    }

    if(back < 50){
      float t = (50 - front) / (50 - 30);
      t = constrain(t, 0.0f, 1.0f);

      float scale = exp(-1 * t);
      vy = (int16_t)round(vy * scale);
    }
  }
}



float normalizeAngle180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

bool btnPressed(int pin) {
  static uint32_t last[40] = {0};

  if (digitalRead(pin) == LOW &&
      millis() - last[pin] > 200) {
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
    float dt =
        (now - ballField.last_update_us) *
        0.000001f;

    dt = constrain(dt, 0.005f, 0.1f);

    float delta =
        normalizeAngle180(
            ball_angle - ballField.last_angle
        );

    float measured_rate = delta / dt;

    ballField.angle_rate =
        0.75f * ballField.angle_rate +
        0.25f * measured_rate;

    ballField.last_angle = ball_angle;
    ballField.last_update_us = now;
  }

  const float look_ahead_seconds = 0.07f;

  float predicted_angle =
      ball_angle +
      ballField.angle_rate *
          look_ahead_seconds;

  ballField.predicted_error =
      normalizeAngle180(
          predicted_angle - 90.0f
      );

  if (!ballField.capture_mode &&
      fabs(ballField.predicted_error) < 18.0f) {
    ballField.capture_mode = true;
  } else if (
      ballField.capture_mode &&
      fabs(ballField.predicted_error) > 25.0f) {
    ballField.capture_mode = false;
  }
}

void applyCaptureControl(
    int16_t &vx,
    int16_t &vy) {

  const float speed = 70.0f;
  const float kp = 0.75f;
  const float kd = 0.2f;
  const float max_lateral = 50.0f;

  float lateral =
      -(kp * ballField.predicted_error +
        kd * ballField.angle_rate);

  lateral = constrain(
      lateral,
      -max_lateral,
      max_lateral
  );

  float forward_squared =
      speed * speed -
      lateral * lateral;

  float forward =
      sqrtf(fmaxf(
          0.0f,
          forward_squared
      ));

  vx = (int16_t)roundf(lateral);
  vy = (int16_t)roundf(forward);
}

void applyBallVectorField(
    int16_t &vx,
    int16_t &vy) {

  float angle = maixPosData.ball_angle;
  float distance = maixPosData.ball_dist;
  float radians = angle * DtoR_const;

  const float reverse_deadband = 20.0f;

  if (ballField.predicted_error >
      reverse_deadband) {
    ballField.orbit_side = 1;
  } else if (
      ballField.predicted_error <
      -reverse_deadband) {
    ballField.orbit_side = -1;
  }

  float radial_x = cosf(radians);
  float radial_y = sinf(radians);

  float tangent_x =
      -ballField.orbit_side * radial_y;

  float tangent_y =
      ballField.orbit_side * radial_x;

  const float target_distance = 65.0f;

  float radial_speed =
      (distance - target_distance) *
      2.2f;

  radial_speed = constrain(
      radial_speed,
      -25.0f,
      70.0f
  );

  float near_ratio =
      (70.0f - distance) / 20.0f;

  near_ratio = constrain(
      near_ratio,
      0.0f,
      1.0f
  );

  float tangent_speed =
      50.0f +
      30.0f * near_ratio;

  float front_blend =
      1.0f -
      fabs(ballField.predicted_error) /
          35.0f;

  front_blend = constrain(
      front_blend,
      0.0f,
      1.0f
  );

  tangent_speed *=
      1.0f - front_blend;

  radial_speed =
      radial_speed *
          (1.0f - front_blend) +
      70.0f * front_blend;

  float command_x =
      radial_speed * radial_x +
      tangent_speed * tangent_x;

  float command_y =
      radial_speed * radial_y +
      tangent_speed * tangent_y;

  const float max_speed = 70.0f;

  float magnitude =
      sqrtf(
          command_x * command_x +
          command_y * command_y
      );

  if (magnitude > max_speed) {
    float scale =
        max_speed / magnitude;

    command_x *= scale;
    command_y *= scale;
  }

  vx = (int16_t)roundf(command_x);
  vy = (int16_t)roundf(command_y);
}

void applyIRChase(
    int16_t &vx,
    int16_t &vy) {

  float radians =
      ballData.angle * DtoR_const;

  float speed = constrain(
      map(
          ballData.dist,
          3,
          1,
          50,
          70
      ),
      50.0f,
      90.0f
  );

  vx = (int16_t)roundf(
      speed * cosf(radians)
  );

  vy = (int16_t)roundf(
      speed * sinf(radians)
  );
}

void applyOmniEdgeBrake(
    int16_t &vx,
    int16_t &vy) {

  if (!maixPosData.valid) {
    return;
  }

  float x = maixPosData.x;
  float y = maixPosData.y;

  if (vx > 0) {
    if (x >= RIGHT_STOP_X) {
      vx = 0;
      return;
    }

    if (x >= RIGHT_SLOW_X) {
      float t =
          (x - RIGHT_SLOW_X) /
          (RIGHT_STOP_X -
           RIGHT_SLOW_X);

      t = constrain(
          t,
          0.0f,
          1.0f
      );

      float scale =
          exp(
              -SIDE_SLOW_EXP * t
          );

      vx = (int16_t)round(
          vx * scale
      );
    }

    return;
  }

  if (vx < 0) {
    if (x <= LEFT_STOP_X) {
      vx = 0;
      return;
    }

    if (x <= LEFT_SLOW_X) {
      float t =
          (LEFT_SLOW_X - x) /
          (LEFT_SLOW_X -
           LEFT_STOP_X);

      t = constrain(
          t,
          0.0f,
          1.0f
      );

      float scale =
          exp(
              -SIDE_SLOW_EXP * t
          );

      vx = (int16_t)round(
          vx * scale
      );
    }

    return;
  }

  if (vy > 0) {
    if (y >= FRONT_STOP_Y) {
      vy = 0;
      return;
    }

    if (y >= FRONT_SLOW_Y) {
      float t =
          (y - FRONT_SLOW_Y) /
          (FRONT_STOP_Y -
           FRONT_SLOW_Y);

      t = constrain(
          t,
          0.0f,
          1.0f
      );

      float scale = exp(-1 * t);

      vy = (int16_t)round(
          vy * scale
      );
    }

    return;
  }

  if (vy < 0) {
    if (y >= BACK_STOP_Y) {
      vy = 0;
      return;
    }

    if (y >= BACK_SLOW_Y) {
      float t =
          (y - BACK_SLOW_Y) /
          (BACK_STOP_Y -
           BACK_SLOW_Y);

      t = constrain(
          t,
          0.0f,
          1.0f
      );

      float scale = exp(-1 * t);

      vy = (int16_t)round(
          vy * scale
      );
    }

    return;
  }
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

void updateKalmanPosition() {
  // 卡爾曼濾波狀態
  static bool initialized = false;
  static float est_x = 0.0f;
  static float est_y = 0.0f;
  static float unc_x = 999.0f;
  static float unc_y = 999.0f;
  static uint32_t last_time = 0;

  // 數值越小，越相信該感測器
  const float CAMERA_NOISE = 0.05f;
  const float ULTRASONIC_NOISE = 0.20f;
  const float PROCESS_NOISE = 0.01f;

  bool us_x_valid =
      isValidUS(us_dist_cm[US_LEFT]) &&
      isValidUS(us_dist_cm[US_RIGHT]);

  bool us_y_valid =
      isValidUS(us_dist_cm[US_FRONT]) &&
      isValidUS(us_dist_cm[US_BACK]);

  float us_x = 0.0f;
  float us_y = 0.0f;

  if (us_x_valid) {
    us_x = (
        us_dist_cm[US_LEFT] -
        us_dist_cm[US_RIGHT]
    ) * 0.5f;
  }

  if (us_y_valid) {
    us_y = (
        us_dist_cm[US_BACK] -
        us_dist_cm[US_FRONT]
    ) * 0.5f;
  }

  bool camera_valid = maixPosData.valid;

  // 第一次取得有效位置時初始化
  if (!initialized) {
    if (camera_valid) {
      est_x = (float)maixPosData.x;
      est_y = (float)maixPosData.y;
    } else if (us_x_valid && us_y_valid) {
      est_x = us_x;
      est_y = us_y;
    } else {
      filtered_pos_valid = false;
      return;
    }

    initialized = true;
    last_time = micros();
  }

  // Predict
  uint32_t now = micros();
  float dt = (now - last_time) * 1e-6f;
  last_time = now;

  dt = constrain(dt, 0.001f, 0.1f);

  unc_x += PROCESS_NOISE * dt;
  unc_y += PROCESS_NOISE * dt;

  // 融合單一座標軸
  auto fuseAxis = [](
      float sensor_value,
      float sensor_noise,
      float &estimate,
      float &uncertainty
  ) {
    float gain =
        uncertainty /
        (uncertainty + sensor_noise);

    estimate +=
        gain * (sensor_value - estimate);

    uncertainty *= 1.0f - gain;
  };

  // 超音波更新
  if (us_x_valid) {
    fuseAxis(
        us_x,
        ULTRASONIC_NOISE,
        est_x,
        unc_x
    );
  }

  if (us_y_valid) {
    fuseAxis(
        us_y,
        ULTRASONIC_NOISE,
        est_y,
        unc_y
    );
  }

  // 相機更新
  if (camera_valid) {
    fuseAxis(
        (float)maixPosData.x,
        CAMERA_NOISE,
        est_x,
        unc_x
    );

    fuseAxis(
        (float)maixPosData.y,
        CAMERA_NOISE,
        est_y,
        unc_y
    );
  }

  filtered_pos_x = est_x;
  filtered_pos_y = est_y;
  filtered_pos_valid = true;
}
  
void setup() {
  Robot_Init();
  setupUS();
  pinMode(
      EAT_BALL_IR_PIN,
      INPUT
  );

  pinMode(COM1, INPUT);
  pinMode(COM2, INPUT);
/*
  ESC.attach(
      A17,
      1000,
      2000
  );

  ESC.writeMicroseconds(1000);
  delay(1500);

  for (int i = 0; i <= 20; i++) {
    ESC.writeMicroseconds(1000);
    delay(20);
  }*/
}


int8_t calculateAimOffset() {
  if (!filtered_pos_valid) {
    return 0;
  }

  const float GOAL_Y = 121.5f;

  // 左半場瞄準右門柱，右半場瞄準左門柱
  float goal_x =
      (filtered_pos_x < 0.0f)
      ? 30.0f
      : -30.0f;

  float dx = goal_x - filtered_pos_x;
  float dy = GOAL_Y - filtered_pos_y;

  float absolute_heading =
      atan2f(dy, dx) * 180.0f / PI;

  // 扣掉正前方90°，轉成 -90～90 的偏移
  float aim_offset =
      absolute_heading - 90.0f;

  aim_offset = constrain(
      aim_offset,
      -90.0f,
      90.0f
  );

  return (int8_t)roundf(aim_offset);
}
void loop() {
  bool attack_enable =
      digitalRead(COM1) == HIGH &&
      digitalRead(COM2) == HIGH;

  // 雙 HIGH 才允許攻擊。
  // 雙 LOW 或一高一低都停止。
  if (!attack_enable) {
    bool was_attack_active =
        com_attack_active;

    com_attack_active = false;

    if (state == ATTACK) {
      state = READY;
    }

    resetBallField();

    cornerShotState =
        CORNER_IDLE;

    kicker_control(false);
    //ESC.writeMicroseconds(1000);

    // 剛停止時立刻送 STOP，
    // 之後每 100ms 重送。
    if (was_attack_active ||
        millis() -
                last_stop_command_ms >=
            100) {

      last_stop_command_ms =
          millis();

      Serial8.write(CMD_STOP);
    }
  }

  // 雙 HIGH 時自動進入 ATTACK。
  if (attack_enable &&
      !com_attack_active &&
      state == READY) {

    com_attack_active = true;
    state = ATTACK;

    Serial8.write(CMD_ATTACK);
    drawState("ATTACK");

    delay(100);
    return;
  }

  if (state == READY) {
    if (btnPressed(BTN_ENTER)) {
      state = SCANNING;

      Serial8.write(
          CMD_LINECAL_START
      );

      drawState("SCANNING");
      return;
    }

    if (attack_enable &&
        btnPressed(BTN_UP)) {

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
      Serial8.write(
          CMD_LINECAL_SAVE
      );

      state = READY;

      uint32_t start = millis();

      while (millis() - start <
             2000) {

        if (Serial8.available() &&
            Serial8.read() ==
                CMD_LINECAL_DONE) {
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
  updateKalmanPosition();
  Serial.print("x= ");Serial.print(filtered_pos_x);
  Serial.print(" y= ");Serial.print(filtered_pos_y);
  
 // FrontCam();if(frontcam.valid){Serial.println("front");};
  
  eat_ball =
      digitalRead(
          EAT_BALL_IR_PIN
      ) == LOW;

  //ESC.writeMicroseconds(1220);

  int16_t vx = 0;
  int16_t vy = 0;
  int8_t aim_offset = 0;

static uint32_t eat_ball_start = 0;
static bool was_eating_ball = false;
static bool kick_sent = false;

    if (eat_ball) {
      resetBallField();

  // 剛吃到球，開始計時
  if (!was_eating_ball) {
    eat_ball_start = millis();
    kick_sent = false;
  }

  was_eating_ball = true;

  vx = 0;
  vy = 50;

      kicker_control(true);
      /*
    if (eat_ball) {
      resetBallField();
      FrontCam();

      if (frontcam.valid) {
        float move_offset = 
            -frontcam.offset 
        ;

        // 正前方是 90 度
        float move_angle = 90.0f + move_offset;
        float radians = move_angle * DtoR_const;
        float speed = 80.0f;

        vx = (int16_t)roundf(speed * cosf(radians));
        vy = (int16_t)roundf(speed * sinf(radians));

        aim_offset = 0;  // 不轉頭
      } else {
        vx = 0;
        vy = 50;
        aim_offset = 0;
      }*/

      //kicker_control(true);
    } else {
      kicker_control(false);
      if (ballData.valid) {
        if (maixPosData.valid &&
            maixPosData.ball_found) {

          updateBallPrediction(
              maixPosData.ball_angle
          );

          if (ballField.capture_mode) {
            applyCaptureControl(
                vx,
                vy
            );
          } else {
            applyBallVectorField(
                vx,
                vy
            );
          }
        } else {
          resetBallField();
          applyIRChase(vx, vy);
        }
      } else {
        resetBallField();
      }
    }
  

  applyOmniEdgeBrake(vx, vy);
  applySideUSBrake(vy);
  applySideUS(vx);
  aim_offset =constrain(
    calculateAimOffset(),-50,50);
    if(filtered_pos_x <40&&filtered_pos_x>-40){aim_offset=0;}
   Serial.print(" offset ");Serial.println(aim_offset);
  sendMovePacket(
      vx,
      vy,
      aim_offset
  );
}