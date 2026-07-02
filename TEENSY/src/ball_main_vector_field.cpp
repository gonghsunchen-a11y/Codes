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

// SLOW：從這個座標開始減速；STOP：到這個座標時推回力最強。
// 把 SLOW 往場中央移會更安全；把 STOP 往外移可用範圍較大但更容易出界。
#define RIGHT_SLOW_X 55.0f
#define RIGHT_STOP_X 70.0f
#define LEFT_SLOW_X -55.0f
#define LEFT_STOP_X -70.0f
#define FRONT_SLOW_Y 70.0f
#define FRONT_STOP_Y 90.0f
#define BACK_SLOW_Y -70.0f
#define BACK_STOP_Y -90.0f
#define SIDE_SLOW_EXP 2.0f

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

enum MainState { READY, SCANNING, ATTACK };
MainState state = READY;

bool eat_ball = false;


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
    // 角速度濾波：0.75 是舊資料、0.25 是新資料，兩者要加起來等於 1。
    // 舊資料比例調大會更穩但反應慢；新資料比例調大會更靈敏但更容易抖。
    ballField.angle_rate =
        0.75f * ballField.angle_rate + 0.25f * measured_rate;

    ballField.last_angle = ball_angle;
    ballField.last_update_us = now;
  }

  // 預測提前量（秒）：調大會更早預判球的位置，但太大容易預測過頭。
  // 建議先在 0.05～0.15 之間調整。
  const float look_ahead_seconds = 0.07f;
  float predicted_angle =
      ball_angle + ballField.angle_rate * look_ahead_seconds;
  ballField.predicted_error =
      normalizeAngle180(predicted_angle - 90.0f);

  // 球與正前方誤差小於 18 度就停止繞球、進入直線收球。
  // 18 調大：更早直衝；調小：要對得更準才直衝。
  if (!ballField.capture_mode &&
      fabs(ballField.predicted_error) < 14.0f ) {
    ballField.capture_mode = true;
  // 已進入收球後，誤差大於 32 度才重新繞球。
  // 32 必須大於上面的 18，否則模式會在臨界角度一直切換。
  } else if (ballField.capture_mode &&
             fabs(ballField.predicted_error) > 25.0f) {
    ballField.capture_mode = false;
  }
}

void applyCaptureControl(int16_t &vx, int16_t &vy) {
  const float speed = 80.0f;       // 收球合成速度：大=追得快，小=比較穩。
  const float kp = 0.75f;           // 角度修正力：大=轉向強，但太大會左右震盪。
  const float kd = 0.15f;          // 抑制快速偏移：大=煞得強，但太大會受雜訊影響。
  const float max_lateral = 50.0f; // 最大左右速度：大=救偏球更強，小=走得更直。

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

  const float reverse_deadband = 20.0f;

if (ballField.predicted_error > reverse_deadband) {
  ballField.orbit_side = 1;
} else if (ballField.predicted_error < -reverse_deadband) {
  ballField.orbit_side = -1;
}

  float radial_x = cosf(radians);
  float radial_y = sinf(radians);
  float tangent_x = -ballField.orbit_side * radial_y;
  float tangent_y = ballField.orbit_side * radial_x;

  // 想保持的繞球半徑，不是「開始繞球的距離」。
  // 調大：離球較遠繞；調小：更貼近球，但更容易碰到球。
  const float target_distance = 65.0f;

  // 2.2 是半徑修正增益：大=快速回到目標半徑，但可能前後震盪。
  float radial_speed = (distance - target_distance) * 2.2f;

  // -15：太近時最大退球速度；55：太遠時最大接近速度。
  // 把 -15 改得更負會更積極避開球；把 55 調大會更快接近球。
  radial_speed = constrain(radial_speed, -25.0f, 70.0f);

  // 距離 75 以上 near_ratio=0；距離 55 以下 near_ratio=1。
  // 75 調大會更早加強繞球；20 調大會讓速度變化更平緩。
  float near_ratio = (70.0f - distance) / 20.0f;
  near_ratio = constrain(near_ratio, 0.0f, 1.0f);

  // 遠處切向速度是 20，靠近後最多再加 50，所以最大約 70。
  // 第一個值控制遠距離繞球速度；第二個值控制靠近後增加多少。
  float tangent_speed = 50.0f + 30.0f * near_ratio;

  // 球進入正前方 30 度範圍後，逐漸由繞球切換成向球前進。
  // 30 調大：更早向球切入；調小：繞到更正才切入。
  float front_blend =
      1.0f - fabs(ballField.predicted_error) / 40.0f;
  front_blend = constrain(front_blend, 0.0f, 1.0f);

  tangent_speed *= 1.0f - front_blend;
  // 完全對準時，將朝球速度混合到 60；調大會更快直衝球。
  radial_speed =
      radial_speed * (1.0f - front_blend) + 70.0f * front_blend;

  float command_x =
      radial_speed * radial_x + tangent_speed * tangent_x;
  float command_y =
      radial_speed * radial_y + tangent_speed * tangent_y;

  // 繞球模式的最大合成速度；調大較快但容易打滑或繞過頭。
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
  // ESP32 備援追球速度：dist=6 時約 40，dist=2 時約 70。
  // 40 是遠／弱訊號時的最低速度；70 是近／強訊號時的最高速度。
  float speed = constrain(map(ballData.dist,3,1,50,70),50.0f,90.0f);

  vx = (int16_t)roundf(speed * cosf(radians));
  vy = (int16_t)roundf(speed * sinf(radians));
}

void applyOmniEdgeBrake(int16_t &vx,int16_t &vy){
  //Serial.print("x");Serial.print(maixPosData.x);
  if(!maixPosData.valid ){
    return;
  }

  float x = maixPosData.x;
  float y = maixPosData.y;
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
  if(vy>0){  if(y >= FRONT_STOP_Y){
      vy = 0;
      return;
    }

    if(y >= FRONT_SLOW_Y){
      float t = (y -FRONT_SLOW_Y) / (FRONT_STOP_Y - FRONT_SLOW_Y);
      t = constrain(t, 0.0f, 1.0f);

      float scale = exp(-1 * t);
      vy = (int16_t)round(vy * scale);
    }

    return;
  }
  if(vy<0){  if(y >= BACK_STOP_Y){
      vy = 0;
      return;
    }

    if(y >= BACK_SLOW_Y){
      float t = (y -BACK_SLOW_Y) / (BACK_STOP_Y - BACK_SLOW_Y);
      t = constrain(t, 0.0f, 1.0f);

      float scale = exp(-1* t);
      vy = (int16_t)round(vy * scale);
    }

    return;
  }
}

/*
void applyBoundaryVectorField(int16_t &vx, int16_t &vy) {
  if (!maixPosData.valid) return;

  float x = maixPosData.x;
  float y = maixPosData.y;
  float out_x = vx;
  float out_y = vy;
  // 邊界推回強度：調大較不易出界，但會更明顯干擾追球路徑。
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
*/
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
bool handleCornerShot(
    bool eatBall,
    int16_t &vx,
    int16_t &vy,
    int8_t &aimOffset) {

  bool inCorner =
      maixPosData.valid &&
      maixPosData.y > 70.0f &&
      fabsf(maixPosData.x) > 65.0f;
  if (cornerShotState != CORNER_IDLE && !eatBall) {
  cornerShotState = CORNER_IDLE;
  kicker_control(false);
  return false;
  }
  // 在角落吃到球才啟動
  if (cornerShotState == CORNER_IDLE) {
    if (!eatBall || !inCorner) {
      return false;
    }

    cornerShotState = CORNER_BACKUP;
    cornerShotTimer = millis();
  }

  switch (cornerShotState) {
    case CORNER_BACKUP:
      // 慢慢後退
      vx = 0;
      vy = -30;
      aimOffset = 0;
      kicker_control(false);

      // 後退 700 ms
      if (millis() - cornerShotTimer >= 700) {
        cornerShotState = CORNER_AIM;
        cornerShotTimer = millis();
      }
      break;

    case CORNER_AIM:
      vx = 0;
      vy = 0;
      kicker_control(false);

      FrontCam();

      if (frontcam.valid) {
        aimOffset = (int8_t)constrain(
            frontcam.offset * 1.5f,
            -60.0f,
            60.0f
        );

        // 球門偏差小於 6，開始射門
        if (fabsf(frontcam.offset) < 6.0f) {
          cornerShotState = CORNER_SHOOT;
          cornerShotTimer = millis();
        }
      } else {
        aimOffset = 0;
      }

      // 避免相機一直無法對準而卡死
      if (millis() - cornerShotTimer >= 2500) {
        cornerShotState = CORNER_SHOOT;
        cornerShotTimer = millis();
      }
      break;

    case CORNER_SHOOT:
      vx = 0;
      vy = 35;
      aimOffset = 0;
      kicker_control(true);

      if (millis() - cornerShotTimer >= 300) {
        kicker_control(false);
        cornerShotState = CORNER_WAIT_BALL_LEAVE;
      }
      break;

    case CORNER_WAIT_BALL_LEAVE:
      vx = 0;
      vy = 0;
      aimOffset = 0;
      kicker_control(false);

      // 球射出去後才允許下一次啟動
      if (!eatBall) {
        cornerShotState = CORNER_IDLE;
      }
      break;

    default:
      cornerShotState = CORNER_IDLE;
      return false;
  }

  return true; // 角落射門正在接管控制
}
void setup() {
  Robot_Init();
  pinMode(EAT_BALL_IR_PIN, INPUT);
/*
  ESC.attach(A17, 1000, 2000);
  ESC.writeMicroseconds(1000);
  delay(2000);

  for (int signal = 1000; signal <= 1500; signal += 10) {
    ESC.writeMicroseconds(signal);
    delay(20);
  }

  ESC.writeMicroseconds(1500);
  delay(5000);
  */
  ESC.attach(A17, 1000, 2000);
  ESC.writeMicroseconds(1000);
  delay(1500);
  for (int i = 0; i <= 20; i++) {
    ESC.writeMicroseconds(1000);
    delay(20);
  }
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
  Serial.print("eat");
  // 吸球馬達 ESC 脈波：調大通常吸力更強，但耗電、發熱也會增加。
  // 建議每次只增加 10～25 us，並確認 ESC 與馬達安全範圍。
  ESC.writeMicroseconds(1220);

  int16_t vx = 0;
  int16_t vy = 0;
  int8_t aim_offset = 0;
  bool cornerControl =
    handleCornerShot(eat_ball, vx, vy, aim_offset);
  if (!cornerControl) {
    if (eat_ball) {
      resetBallField();
      vx = 0;
      FrontCam();
    
    if (frontcam.valid) {
      // 1.5 是球門瞄準增益：大=轉得快但容易震盪；小=平穩但對準較慢。
      // -45～45 是最大旋轉修正範圍，放大會允許更激烈的轉向。
      aim_offset =(int8_t)constrain(frontcam.offset * 1.5f, -60.0f, 60.0f);
      vy = 80;
    }
    else{
      vy = 50;
      aim_offset =0;
    }

    kicker_control(true);
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
}
  //Serial.print("x= ");Serial.print(maixPosData.x);
  //Serial.print(" y=");Serial.println(maixPosData.y);
  //applyBoundaryVectorField(vx, vy);
  applyOmniEdgeBrake(vx,vy);
  sendMovePacket(vx, vy, aim_offset);
}
