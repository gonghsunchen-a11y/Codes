/*
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
#define RIGHT_STOP_X 80.0f
#define LEFT_SLOW_X -55.0f
#define LEFT_STOP_X -80.0f
#define FRONT_SLOW_Y 40.0f
#define FRONT_STOP_Y 70.0f
#define BACK_SLOW_Y -40.0f
#define BACK_STOP_Y -65.0f
#define SIDE_SLOW_EXP 4.0
#define SIDE_SLOW_DIST 80.0f
#define SIDE_STOP_DIST 60.0f
#define SIDE_DANGER_DIST 50.0f
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

float filtered_pos_x = 0.0f;
float filtered_pos_y = 0.0f;
bool filtered_pos_valid = false;


enum MainState { READY, SCANNING, ATTACK };
MainState state = READY;

bool eat_ball = false;
bool com_attack_active = false;
uint32_t last_stop_command_ms = 0;

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

void applySideUS(int16_t &vx,int16_t &vy){
  float left = us_dist_cm[US_LEFT];
  float right = us_dist_cm[US_RIGHT];
  float front = us_dist_cm[US_FRONT];
  float back = us_dist_cm[US_BACK];
  if(isValidUS(left)&&vx<0){
    

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
  }
  if(isValidUS(right)){

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
  if(isValidUS(front)){

    if(vy > 0){
      if(front <= SIDE_STOP_DIST){
        vy = 0;
      }
      else if(front < SIDE_SLOW_DIST){
        float t = (SIDE_SLOW_DIST - front) / (SIDE_SLOW_DIST - SIDE_STOP_DIST);
        t = constrain(t, 0.0f, 1.0f);

        float scale = exp(-SIDE_SLOW_EXP * t);
        vx = (int16_t)round(vy * scale);
      }
    }
  }
  if(isValidUS(back)){

    if(vy < 0){
      if(back <= SIDE_STOP_DIST){
        vy = 0;
      }
      else if(back < SIDE_SLOW_DIST){
        float t = (SIDE_SLOW_DIST - back) / (SIDE_SLOW_DIST - SIDE_STOP_DIST);
        t = constrain(t, 0.0f, 1.0f);

        float scale = exp(-SIDE_SLOW_EXP * t);
        vy = (int16_t)round(vy * scale);
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
    
    if(maixPosData.y <=20){
      return; 
    }
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
      float t = (50 - back) / (50 - 30);
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
      fabs(ballField.predicted_error) < 14.0f) {
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

  const float speed = 50.0f;
  const float kp = 0.75f;
  const float kd = 0.35f;
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
      -20.0f,
      50.0f
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
          45.0f;

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
      90.0f * front_blend;

  float command_x =
      radial_speed * radial_x +
      tangent_speed * tangent_x;

  float command_y =
      radial_speed * radial_y +
      tangent_speed * tangent_y;

  const float max_speed = 80.0f;

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
          90
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

  if (!filtered_pos_valid) {
    return;
  }

  float x = filtered_pos_x;
  float y = filtered_pos_y;

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
  ESC.attach(A17, 1000, 2000);
  ESC.writeMicroseconds(1000);
  delay(1500);
  for (int i = 0; i <= 20; i++) {
    ESC.writeMicroseconds(1000);
    delay(20);
  }
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

    kicker_control(false);
    ESC.writeMicroseconds(1000);

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

  //readBNO085Yaw();
  ballsensor();
  updateUS();
  //readMaix();
  //updateKalmanPosition();
  readGoal();
  if(maixPosData.valid){Serial.println("yes");}
  
  //Serial.print("x= ");Serial.print(filtered_pos_x);
  //Serial.print(" y= ");Serial.println(filtered_pos_y);
 // FrontCam();if(frontcam.valid){Serial.println("front");};
  
  eat_ball =
      digitalRead(
          EAT_BALL_IR_PIN
      ) == LOW;

  ESC.writeMicroseconds(1220);

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
  int8_t x = goalData.angle;
  vx = 0;
  if(goalData.valid){
    vy = 80;
  }
  else{
  vy = 50;
  }
    if(goalData.angle>90){
      aim_offset = 25;
    }
    else{
      aim_offset = -25;
    }

    if(!kick_sent && millis() - eat_ball_start > 75UL){
      kicker_control(1);
      kick_sent = true;
      
    }else{
      kicker_control(0);
      aim_offset =0;
    }
    if(filtered_pos_x>65&&filtered_pos_y>80||filtered_pos_x<-65&&filtered_pos_y>80){
      vy=10;
      aim_offset=0;
      kicker_control(0);
    }
    }
     else {
      was_eating_ball = false;
      eat_ball_start = 0;
      kick_sent = false;

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
   if (ballData.valid) {
  if (maixPosData.valid) {

    float diff = fabs(normalizeAngle180(
        ballData.angle - maixPosData.ball_angle
    ));

    if (diff > 180.0f) {
      resetBallField();
      applyIRChase(vx, vy);  // 角度差大，優先紅外線
    } else {
      updateBallPrediction(maixPosData.ball_angle);

      if (ballField.capture_mode) {
        applyCaptureControl(vx, vy);
      } else {
        applyBallVectorField(vx, vy);
      }
    }

  } else {
    resetBallField();
    applyIRChase(vx, vy);
  }
} else {
  resetBallField();
  if (filtered_pos_valid) {
    const float KP = 0.8f;
    const float MAX_SPEED = 50.0f;
    const float DEADBAND = 5.0f;

    vx = (fabs(filtered_pos_x) > DEADBAND)
        ? constrain(-filtered_pos_x * KP, -MAX_SPEED, MAX_SPEED)
        : 0;

    vy = (fabs(filtered_pos_y) > DEADBAND)
        ? constrain(-filtered_pos_y * KP, -MAX_SPEED, MAX_SPEED)
        : 0;
  } else {
    vx = 0;
    vy = 0;
  }

  aim_offset = 0;
}
    }
  

  //applyOmniEdgeBrake(vx, vy);
  //applySideUSBrake(vy);
  applySideUS(vx,vy);
   Serial.print(" l ");Serial.println(us_dist_cm[US_LEFT]);
    Serial.print(" f ");Serial.println(us_dist_cm[US_FRONT]);
     Serial.print(" r ");Serial.println(us_dist_cm[US_RIGHT]);
      Serial.print(" b ");Serial.println(us_dist_cm[US_BACK]);
    if(filtered_pos_x <40&&filtered_pos_x>-40){aim_offset=0;}
   Serial.print(" offset ");Serial.println(aim_offset);
  sendMovePacket(
      vx,
      vy,
      aim_offset
  );
}*/

#include <Arduino.h>
#include <Robot.h>
#include <Servo.h>
#include <Wire.h>

#include <math.h>

Servo ESC;

// ==================== 通訊命令 ====================

#define CMD_ATTACK        0xAA
#define CMD_LINECAL_START 0xCC
#define CMD_LINECAL_SAVE  0xEE
#define CMD_LINECAL_DONE  0xDD
#define CMD_STOP          0xBB

// ==================== 腳位 ====================

#define COM1 36
#define COM2 37

#define EAT_BALL_IR_PIN A16

#define TRIG_F 2
#define ECHO_F 6
#define TRIG_R 3
#define ECHO_R 8
#define TRIG_B 4
#define ECHO_B 9
#define TRIG_L 5
#define ECHO_L 10

// ==================== 超音波設定 ====================

#define US_COUNT 4
#define US_INVALID_DISTANCE 999.0f
#define US_FILTER_ALPHA 0.35f
#define US_INVALID_LIMIT 3

#define SIDE_SLOW_DIST 80.0f
#define SIDE_STOP_DIST 60.0f
#define SIDE_SLOW_EXP 4.0f

// ==================== 球控制設定 ====================

#define SENSOR_MAX_DIFFERENCE 100.0f

#define ALIGN_ENTER_ANGLE 18.0f
#define ALIGN_EXIT_ANGLE 35.0f
#define ALIGN_HOLD_TIME 120UL

#define BALL_TARGET_DISTANCE 65.0f
#define CAPTURE_SPEED 65.0f
#define MAX_ORBIT_SPEED 65.0f

#define DEG_TO_RAD_F 0.017453292519943295f

// ==================== 狀態 ====================

enum MainState {
  READY,
  SCANNING,
  ATTACK
};

MainState state = READY;

bool eat_ball = false;
bool com_attack_active = false;

uint32_t last_stop_command_ms = 0;

// ==================== 相機球資料 ====================

struct CameraBallData {
  bool valid = false;
  uint16_t angle = 0;
  uint16_t dist = 0;
  uint32_t last_update = 0;
};

CameraBallData cameraBall;

// ==================== 繞球狀態 ====================

struct BallFieldState {
  int8_t orbit_side = 0;

  bool capture_mode = false;
  bool angle_initialized = false;

  float last_angle = 90.0f;
  float angle_rate = 0.0f;

  // 球角度與球門角度的誤差
  float predicted_error = 0.0f;

  uint32_t last_update_us = 0;
  uint32_t aligned_start_ms = 0;
};

BallFieldState ballField;

// ==================== 超音波資料 ====================

enum USIndex {
  US_FRONT = 0,
  US_RIGHT = 1,
  US_BACK = 2,
  US_LEFT = 3
};

const uint8_t trigPins[US_COUNT] = {
  TRIG_F,
  TRIG_R,
  TRIG_B,
  TRIG_L
};

const uint8_t echoPins[US_COUNT] = {
  ECHO_F,
  ECHO_R,
  ECHO_B,
  ECHO_L
};

volatile uint32_t echo_start[US_COUNT] = {0};
volatile uint32_t echo_duration[US_COUNT] = {0};
volatile bool echo_done[US_COUNT] = {false};

float us_dist_cm[US_COUNT] = {
  US_INVALID_DISTANCE,
  US_INVALID_DISTANCE,
  US_INVALID_DISTANCE,
  US_INVALID_DISTANCE
};

uint8_t us_invalid_count[US_COUNT] = {0};

uint8_t current_us = US_COUNT - 1;
uint32_t last_trigger_time = 0;

// ==================================================
// 通用函式
// ==================================================

float normalizeAngle180(float angle) {
  while (angle > 180.0f) {
    angle -= 360.0f;
  }

  while (angle < -180.0f) {
    angle += 360.0f;
  }

  return angle;
}

bool btnPressed(int pin) {
  static uint32_t last[40] = {0};

  if (pin < 0 || pin >= 40) {
    return false;
  }

  if (digitalRead(pin) == LOW &&
      millis() - last[pin] > 200UL) {

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

// ==================================================
// MaixCam 14-byte 合併封包
// ==================================================


 
void readVisionPacket() {
  uint8_t buffer[14];

  goalData.valid = false;
  cameraBall.valid = false;

  // 清除上一包殘留資料
  while (Serial3.available()) {
    Serial3.read();
  }

  // 要求 MaixCam 回傳資料
  Serial3.write(0xDD);

  uint32_t start = millis();

  while (Serial3.available() < 14) {
    if (millis() - start > 30UL) {
      return;
    }
  }

  if (Serial3.readBytes(buffer, 14) != 14) {
    return;
  }

  if (buffer[0] != 0xCC ||
      buffer[1] != 0xDD ||
      buffer[13] != 0xEE) {
    return;
  }

  uint8_t checksum = 0xDD;

  for (uint8_t i = 2; i <= 11; i++) {
    checksum += buffer[i];
  }

  if (checksum != buffer[12]) {
    return;
  }

  uint16_t goal_angle =
      (uint16_t)buffer[3] |
      ((uint16_t)buffer[4] << 8);

  uint16_t goal_dist =
      (uint16_t)buffer[5] |
      ((uint16_t)buffer[6] << 8);

  if (buffer[2] != 0 &&
      goal_angle < 360 &&
      goal_dist != 0xFFFF) {

    goalData.valid = true;
    goalData.angle = goal_angle;
    goalData.dist = goal_dist;
    goalData.last_update = millis();
  }

  uint16_t ball_angle =
      (uint16_t)buffer[8] |
      ((uint16_t)buffer[9] << 8);

  uint16_t ball_dist =
      (uint16_t)buffer[10] |
      ((uint16_t)buffer[11] << 8);

  if (buffer[7] != 0 &&
      ball_angle < 360 &&
      ball_dist != 0xFFFF) {

    cameraBall.valid = true;
    cameraBall.angle = ball_angle;
    cameraBall.dist = ball_dist;
    cameraBall.last_update = millis();
  }
}

// ==================================================
// 超音波
// ==================================================

bool isValidUS(float dist_cm) {
  return dist_cm > 0.0f &&
         dist_cm < US_INVALID_DISTANCE;
}

void echoISR(uint8_t index) {
  if (digitalRead(echoPins[index]) == HIGH) {
    echo_start[index] = micros();
  } else {
    echo_duration[index] =
        micros() - echo_start[index];

    echo_done[index] = true;
  }
}

void echoFrontISR() {
  echoISR(US_FRONT);
}

void echoRightISR() {
  echoISR(US_RIGHT);
}

void echoBackISR() {
  echoISR(US_BACK);
}

void echoLeftISR() {
  echoISR(US_LEFT);
}

void triggerUS(uint8_t index) {
  digitalWrite(trigPins[index], LOW);
  delayMicroseconds(2);

  digitalWrite(trigPins[index], HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPins[index], LOW);
}

void updateFilteredUS(
    uint8_t index,
    float raw_dist_cm) {

  us_invalid_count[index] = 0;

  if (!isValidUS(us_dist_cm[index])) {
    us_dist_cm[index] = raw_dist_cm;
  } else {
    us_dist_cm[index] =
        us_dist_cm[index] *
            (1.0f - US_FILTER_ALPHA) +
        raw_dist_cm *
            US_FILTER_ALPHA;
  }
}

void markInvalidUS(uint8_t index) {
  if (us_invalid_count[index] <
      US_INVALID_LIMIT) {

    us_invalid_count[index]++;
  }

  if (us_invalid_count[index] >=
      US_INVALID_LIMIT) {

    us_dist_cm[index] =
        US_INVALID_DISTANCE;
  }
}

void updateUS() {
  if (millis() - last_trigger_time >= 50UL) {
    last_trigger_time = millis();

    current_us++;

    if (current_us >= US_COUNT) {
      current_us = 0;
    }

    echo_done[current_us] = false;
    triggerUS(current_us);
  }

  for (uint8_t i = 0; i < US_COUNT; i++) {
    if (!echo_done[i]) {
      continue;
    }

    noInterrupts();

    uint32_t duration =
        echo_duration[i];

    echo_done[i] = false;

    interrupts();

    if (duration > 100 &&
        duration < 15000) {

      float distance =
          duration *
          0.0343f /
          2.0f;

      updateFilteredUS(i, distance);
    } else {
      markInvalidUS(i);
    }
  }
}

void setupUS() {
  for (uint8_t i = 0; i < US_COUNT; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);

    digitalWrite(trigPins[i], LOW);
  }

  attachInterrupt(
      digitalPinToInterrupt(ECHO_F),
      echoFrontISR,
      CHANGE
  );

  attachInterrupt(
      digitalPinToInterrupt(ECHO_R),
      echoRightISR,
      CHANGE
  );

  attachInterrupt(
      digitalPinToInterrupt(ECHO_B),
      echoBackISR,
      CHANGE
  );

  attachInterrupt(
      digitalPinToInterrupt(ECHO_L),
      echoLeftISR,
      CHANGE
  );
}

void applySideUS(
    int16_t &vx,
    int16_t &vy) {

  float left =
      us_dist_cm[US_LEFT];

  float right =
      us_dist_cm[US_RIGHT];

  float front =
      us_dist_cm[US_FRONT];

  float back =
      us_dist_cm[US_BACK];

  // 往左移動
  if (vx < 0 &&
      isValidUS(left)) {

    if (left <= SIDE_STOP_DIST) {
      vx = 0;
    } else if (left < SIDE_SLOW_DIST) {
      float t =
          (SIDE_SLOW_DIST - left) /
          (SIDE_SLOW_DIST -
           SIDE_STOP_DIST);

      t = constrain(
          t,
          0.0f,
          1.0f
      );

      float scale =
          expf(
              -SIDE_SLOW_EXP *
              t
          );

      vx = (int16_t)roundf(
          vx * scale
      );
    }
  }

  // 往右移動
  if (vx > 0 &&
      isValidUS(right)) {

    if (right <= SIDE_STOP_DIST) {
      vx = 0;
    } else if (right < SIDE_SLOW_DIST) {
      float t =
          (SIDE_SLOW_DIST - right) /
          (SIDE_SLOW_DIST -
           SIDE_STOP_DIST);

      t = constrain(
          t,
          0.0f,
          1.0f
      );

      float scale =
          expf(
              -SIDE_SLOW_EXP *
              t
          );

      vx = (int16_t)roundf(
          vx * scale
      );
    }
  }

  // 往前移動
  if (vy > 0 &&
      isValidUS(front)) {

    if (front <= SIDE_STOP_DIST) {
      vy = 0;
    } else if (front < SIDE_SLOW_DIST) {
      float t =
          (SIDE_SLOW_DIST - front) /
          (SIDE_SLOW_DIST -
           SIDE_STOP_DIST);

      t = constrain(
          t,
          0.0f,
          1.0f
      );

      float scale =
          expf(
              -SIDE_SLOW_EXP *
              t
          );

      vy = (int16_t)roundf(
          vy * scale
      );
    }
  }

  // 往後移動
  if (vy < 0 &&
      isValidUS(back)) {

    if (back <= SIDE_STOP_DIST) {
      vy = 0;
    } else if (back < SIDE_SLOW_DIST) {
      float t =
          (SIDE_SLOW_DIST - back) /
          (SIDE_SLOW_DIST -
           SIDE_STOP_DIST);

      t = constrain(
          t,
          0.0f,
          1.0f
      );

      float scale =
          expf(
              -SIDE_SLOW_EXP *
              t
          );

      vy = (int16_t)roundf(
          vy * scale
      );
    }
  }
}

// ==================================================
// 球門導向繞球
// ==================================================

void resetBallField() {
  ballField.orbit_side = 0;
  ballField.capture_mode = false;
  ballField.angle_initialized = false;

  ballField.last_angle = 90.0f;
  ballField.angle_rate = 0.0f;
  ballField.predicted_error = 0.0f;

  ballField.last_update_us = 0;
  ballField.aligned_start_ms = 0;
}

void updateBallPrediction(
    float ball_angle,
    float target_angle) {

  uint32_t now_us = micros();

  if (!ballField.angle_initialized) {
    ballField.angle_initialized = true;

    ballField.last_angle =
        ball_angle;

    ballField.last_update_us =
        now_us;

    ballField.angle_rate = 0.0f;

    ballField.predicted_error =
        normalizeAngle180(
            ball_angle -
            target_angle
        );
  } else {
    float dt =
        (now_us -
         ballField.last_update_us) *
        0.000001f;

    dt = constrain(
        dt,
        0.005f,
        0.1f
    );

    float angle_delta =
        normalizeAngle180(
            ball_angle -
            ballField.last_angle
        );

    float measured_rate =
        angle_delta /
        dt;

    // 降低靈敏度
    ballField.angle_rate =
        0.85f *
            ballField.angle_rate +
        0.15f *
            measured_rate;

    ballField.last_angle =
        ball_angle;

    ballField.last_update_us =
        now_us;

    const float LOOK_AHEAD = 0.05f;

    float predicted_angle =
        ball_angle +
        ballField.angle_rate *
            LOOK_AHEAD;

    float raw_error =
        normalizeAngle180(
            predicted_angle -
            target_angle
        );

    // 低通濾波
    float error_delta =
        normalizeAngle180(
            raw_error -
            ballField.predicted_error
        );

    ballField.predicted_error =
        normalizeAngle180(
            ballField.predicted_error +
            error_delta * 0.20f
        );
  }

  float abs_error =
      fabsf(
          ballField.predicted_error
      );

  // 對準120ms才進入直衝
  if (abs_error <= ALIGN_ENTER_ANGLE) {
    if (ballField.aligned_start_ms == 0) {
      ballField.aligned_start_ms =
          millis();
    }

    if (!ballField.capture_mode &&
        millis() -
                ballField.aligned_start_ms >=
            ALIGN_HOLD_TIME) {

      ballField.capture_mode = true;
    }
  } else {
    ballField.aligned_start_ms = 0;
  }

  // 偏差大於35度才重新繞球
  if (ballField.capture_mode &&
      abs_error >= ALIGN_EXIT_ANGLE) {

    ballField.capture_mode = false;
    ballField.aligned_start_ms = 0;
  }
}

void applyCaptureControl(
    int16_t &vx,
    int16_t &vy,
    float ball_angle) {

  float radians =
      ball_angle *
      DEG_TO_RAD_F;

  vx = (int16_t)roundf(
      CAPTURE_SPEED *
      cosf(radians)
  );

  vy = (int16_t)roundf(
      CAPTURE_SPEED *
      sinf(radians)
  );
}

void applyBallVectorField(
    int16_t &vx,
    int16_t &vy) {

  float ball_angle =
      cameraBall.angle;

  float ball_distance =
      cameraBall.dist;

  float radians =
      ball_angle *
      DEG_TO_RAD_F;

  float alignment_error =
      ballField.predicted_error;

  const float SIDE_CHANGE_ANGLE =
      15.0f;

  if (alignment_error >
      SIDE_CHANGE_ANGLE) {

    ballField.orbit_side = 1;
  } else if (
      alignment_error <
      -SIDE_CHANGE_ANGLE) {

    ballField.orbit_side = -1;
  }

  if (ballField.orbit_side == 0) {
    ballField.orbit_side =
        alignment_error >= 0.0f
            ? 1
            : -1;
  }

  // 指向球的單位向量
  float radial_x =
      cosf(radians);

  float radial_y =
      sinf(radians);

  // 繞球切線
  float tangent_x =
      -ballField.orbit_side *
      radial_y;

  float tangent_y =
      ballField.orbit_side *
      radial_x;

  // 維持與球約65像素距離
  float radial_speed =
      (ball_distance -
       BALL_TARGET_DISTANCE) *
      1.6f;

  radial_speed = constrain(
      radial_speed,
      -18.0f,
      35.0f
  );

  // 誤差越大，繞球越快
  float turn_ratio =
      fabsf(alignment_error) /
      90.0f;

  turn_ratio = constrain(
      turn_ratio,
      0.0f,
      1.0f
  );

  float tangent_speed =
      25.0f +
      30.0f *
          turn_ratio;

  // 接近對準時降低切線速度
  float alignment_ratio =
      fabsf(alignment_error) /
      30.0f;

  alignment_ratio = constrain(
      alignment_ratio,
      0.0f,
      1.0f
  );

  tangent_speed *=
      alignment_ratio;

  float command_x =
      radial_speed *
          radial_x +
      tangent_speed *
          tangent_x;

  float command_y =
      radial_speed *
          radial_y +
      tangent_speed *
          tangent_y;

  float magnitude =
      sqrtf(
          command_x * command_x +
          command_y * command_y
      );

  if (magnitude >
      MAX_ORBIT_SPEED) {

    float scale =
        MAX_ORBIT_SPEED /
        magnitude;

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
      ballData.angle *
      DEG_TO_RAD_F;

  float speed =
      (float)map(
          ballData.dist,
          3,
          1,
          50,
          90
      );

  speed = constrain(
      speed,
      50.0f,
      90.0f
  );

  vx = (int16_t)roundf(
      speed *
      cosf(radians)
  );

  vy = (int16_t)roundf(
      speed *
      sinf(radians)
  );
}

void calculateBallMovement(
    int16_t &vx,
    int16_t &vy) {

  // 相機有看到球
  if (cameraBall.valid) {
    // IR和相機都有球時，檢查角度差
    if (ballData.valid) {
      float difference =
          fabsf(
              normalizeAngle180(
                  ballData.angle -
                  cameraBall.angle
              )
          );

      // 兩個感測器差太多，使用IR
      if (difference >
          180) {

        resetBallField();
        applyIRChase(vx, vy);
        return;
      }
    }

    
    // * 有球門時，以球門角度為目標。
    // * 沒球門時，以正前方90度為目標。
     
    float target_angle = 90.0f;

    if (goalData.valid) {
      target_angle =
          goalData.angle;
    }

    updateBallPrediction(
        cameraBall.angle,
        target_angle
    );

    if (ballField.capture_mode) {
      // 球與球門對準，直接朝球衝
      applyCaptureControl(
          vx,
          vy,
          cameraBall.angle
      );
    } else {
      // 尚未對準，繼續繞球
      applyBallVectorField(
          vx,
          vy
      );
    }

    return;
  }

  // 相機沒有球，使用IR
  if (ballData.valid) {
    resetBallField();
    applyIRChase(vx, vy);
    return;
  }

  // 兩邊都沒有球
  resetBallField();

  vx = 0;
  vy = 0;
}

// ==================================================
// 移動封包
// ==================================================

void sendMovePacket(
    int16_t vx,
    int16_t vy,
    int8_t aim_offset) {

  vx = constrain(
      vx,
      -100,
      100
  );

  vy = constrain(
      vy,
      -100,
      100
  );

  aim_offset = constrain(
      aim_offset,
      -100,
      100
  );

  uint8_t packet[7];

  packet[0] = 0xAA;
  packet[1] = 0xAA;

  packet[2] =
      (uint8_t)(int8_t)vx;

  packet[3] =
      (uint8_t)(int8_t)vy;

  packet[4] =
      (uint8_t)aim_offset;

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

// ==================================================
// Setup
// ==================================================

void setup() {
  Robot_Init();
  setupUS();

  pinMode(
      EAT_BALL_IR_PIN,
      INPUT
  );

  pinMode(COM1, INPUT);
  pinMode(COM2, INPUT);

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
  }

  drawState("READY");
}

// ==================================================
// Loop
// ==================================================

void loop() {
  bool attack_enable =
      digitalRead(COM1) == HIGH &&
      digitalRead(COM2) == HIGH;

  // 停止攻擊
  if (!attack_enable) {
    bool was_attack_active =
        com_attack_active;

    com_attack_active = false;

    if (state == ATTACK) {
      state = READY;
    }

    resetBallField();

    kicker_control(false);
    ESC.writeMicroseconds(1000);

    if (was_attack_active ||
        millis() -
                last_stop_command_ms >=
            100UL) {

      last_stop_command_ms =
          millis();

      Serial8.write(CMD_STOP);
    }
  }

  // COM訊號開啟後自動進入攻擊
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

  // 等待狀態
  if (state == READY) {
    if (btnPressed(BTN_ENTER)) {
      state = SCANNING;

      Serial8.write(
          CMD_LINECAL_START
      );

      drawState("SCANNING");
      return;
    }

    drawState("READY");
    return;
  }

  // 白線校正
  if (state == SCANNING) {
    if (btnPressed(BTN_ESC)) {
      Serial8.write(
          CMD_LINECAL_SAVE
      );

      state = READY;

      uint32_t start =
          millis();

      while (millis() - start <
             2000UL) {

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

  // ==================== 攻擊模式 ====================

  ballsensor();
  updateUS();
  readVisionPacket();

  eat_ball =
      digitalRead(
          EAT_BALL_IR_PIN
      ) == LOW;

  ESC.writeMicroseconds(1220);

  int16_t vx = 0;
  int16_t vy = 0;
  int8_t aim_offset = 0;

  static uint32_t eat_ball_start = 0;
  static bool was_eating_ball = false;
  static bool kick_sent = false;

  if (eat_ball) {
    resetBallField();

    if (!was_eating_ball) {
      eat_ball_start = millis();
      kick_sent = false;
    }

    was_eating_ball = true;

    vx = 0;

    if (goalData.valid) {
      vy = 80;

      float goal_error =
          normalizeAngle180(
              goalData.angle -
              90.0f
          );

      if (goal_error > 8.0f) {
        aim_offset = 25;
      } else if (goal_error < -8.0f) {
        aim_offset = -25;
      } else {
        aim_offset = 0;
      }
    } else {
      vy = 50;
      aim_offset = 0;
    }

    if (!kick_sent &&
        millis() -
                eat_ball_start >
            50UL) {

      kicker_control(true);
      kick_sent = true;
    } else {
      kicker_control(false);
    }
  } else {
    was_eating_ball = false;
    eat_ball_start = 0;
    kick_sent = false;

    kicker_control(false);

    calculateBallMovement(
        vx,
        vy
    );
  }

  // 超音波防撞
  //applySideUS(vx, vy);

  sendMovePacket(
      vx,
      vy,
      aim_offset
  );
}