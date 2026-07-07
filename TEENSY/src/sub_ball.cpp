#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>
#include <EEPROM.h>

#define CMD_ATTACK 0xAA
#define CMD_LINECAL_START 0xCC
#define CMD_LINECAL_SAVE 0xEE
#define CMD_LINECAL_DONE 0xDD
#define CMD_STOP 0xBB

enum SubState {
  SUB_IDLE,
  SUB_LINECAL,
  SUB_ATTACK
};

SubState subState = SUB_IDLE;

#define M1 A0
#define M2 A1
#define s0 A2
#define s1 A3
#define s2 A4
#define s3 A5
#define LS_count 32

struct LineData {
  uint32_t state = 0xFFFFFFFF;
} lineData;

uint16_t max_ls[LS_count];
uint16_t avg_ls[LS_count];
uint16_t min_ls[LS_count];

float lineVx = 0;
float lineVy = 0;
float init_lineDegree = -1;
float diff = 0;

bool overhalf = false;
bool first_detect = false;

uint32_t speed_timer = 0;

float vx;
float vy;
int aim_offset;

float finalVx;
float finalVy;

int readMux(int ch, int sigPin) {
  digitalWrite(
      s0,
      (ch >> 0) & 1
  );

  digitalWrite(
      s1,
      (ch >> 1) & 1
  );

  digitalWrite(
      s2,
      (ch >> 2) & 1
  );

  digitalWrite(
      s3,
      (ch >> 3) & 1
  );

  delayMicroseconds(20);

  if (sigPin == 1) {
    return analogRead(M1);
  }

  if (sigPin == 2) {
    return analogRead(M2);
  }

  return 0;
}

void line_calibrate() {
  for (int i = 0; i < LS_count; i++) {
    max_ls[i] = 0;
    min_ls[i] = 4095;
    avg_ls[i] = 0;
  }

  while (1) {
    for (
        uint8_t i = 0;
        i < LS_count;
        i++) {

      avg_ls[i] =
          (max_ls[i] + min_ls[i]) /
          2;
    }

    if (Serial8.available()) {
      if (Serial8.read() ==
          CMD_LINECAL_SAVE) {

        for (
            uint8_t i = 0;
            i < LS_count;
            i++) {

          Serial.print(" min ");
          Serial.print(i);
          Serial.print(" = ");
          Serial.print(min_ls[i]);

          Serial.print(" max ");
          Serial.print(i);
          Serial.print(" = ");
          Serial.print(max_ls[i]);

          Serial.print(" avg ");
          Serial.print(i);
          Serial.print(" = ");
          Serial.print(avg_ls[i]);

          Serial.println("");
        }

        break;
      }
    }

    for (
        uint8_t i = 0;
        i < LS_count;
        i++) {

      uint16_t reading =
          readMux(
              i % 16,
              (i < 16) ? 1 : 2
          );

      if (reading > max_ls[i]) {
        max_ls[i] = reading;
      }

      if (reading < min_ls[i]) {
        min_ls[i] = reading;
      }
    }
  }

  for (
      uint8_t i = 0;
      i < LS_count;
      i++) {

    avg_ls[i] =
        (max_ls[i] + min_ls[i]) /
        2;
  }

  EEPROM.put(0, avg_ls);
  Serial8.write(CMD_LINECAL_DONE);
}

void fast_update_line_sensor() {
  uint32_t rawState =
      0xFFFFFFFF;

  for (
      uint8_t ch = 0;
      ch < 16;
      ch++) {

    digitalWriteFast(
        s0,
        (ch >> 0) & 1
    );

    digitalWriteFast(
        s1,
        (ch >> 1) & 1
    );

    digitalWriteFast(
        s2,
        (ch >> 2) & 1
    );

    digitalWriteFast(
        s3,
        (ch >> 3) & 1
    );

    analogRead(M1);
    uint16_t r1 =
        analogRead(M1);

    analogRead(M2);
    uint16_t r2 =
        analogRead(M2);

    if (r1 < avg_ls[ch]) {
      rawState &=
          ~(1UL << ch);
    }

    if (r2 < avg_ls[ch + 16]) {
      rawState &=
          ~(1UL << (ch + 16));
    }
  }

  lineData.state = rawState;
}

bool moveBackInBounds() {
  float sumX = 0.0f;
  float sumY = 0.0f;

  int count = 0;

  for (
      int i = 0;
      i < LS_count;
      i++) {

    if (bitRead(
            lineData.state,
            i
        ) == 0) {

      float deg =
          linesensorDegreelist[i];

      sumX += cos(
          deg * DtoR_const
      );

      sumY += sin(
          deg * DtoR_const
      );

      count++;
    }
  }

  if (count > 1) {
    float lineDegree =
        atan2(sumY, sumX) *
        RtoD_const;

    if (lineDegree < 0) {
      lineDegree += 360;
    }

    if (!first_detect) {
      init_lineDegree =
          lineDegree;

      first_detect = true;
      speed_timer = millis();
    }

    diff =
        fabs(
            lineDegree -
            init_lineDegree
        );

    if (diff > 180) {
      diff = 360 - diff;
    }

    float finalDegree;

    if (diff >
        EMERGENCY_THRESHOLD) {

      //overhalf = true;

      finalDegree =
          fmod(
              init_lineDegree +
                  180.0f,
              360.0f
          );
    } else {
      //overhalf = false;

      finalDegree =
          fmod(
              lineDegree +
                  180.0f,
              360.0f
          );
    }

    lineVx =
        90.0f *
        cos(
            finalDegree *
            DtoR_const
        );

    lineVy =
        90.0f *
        sin(
            finalDegree *
            DtoR_const
        );

    return true;
  }

  first_detect = false;
  lineVx = 0;
  lineVy = 0;

  return false;
}

void readCommand() {
  while (Serial8.available() > 0) {
    uint8_t cmd =
        Serial8.read();

    Serial.print("cmd=");
    Serial.println(cmd, HEX);

    if (cmd ==
            CMD_LINECAL_START &&
        subState == SUB_IDLE) {

      subState = SUB_LINECAL;

      line_calibrate();

      subState = SUB_IDLE;
      return;
    }

    if (cmd == CMD_ATTACK &&
        subState == SUB_IDLE) {

      Serial.println(
          "ENTER ATTACK"
      );

      subState = SUB_ATTACK;
      return;
    }

    // 在 IDLE 狀態收到 STOP，
    // 確保馬達維持停止。
    if (cmd == CMD_STOP) {
      vx = 0;
      vy = 0;
      aim_offset = 0;

      subState = SUB_IDLE;
      MotorStop();
      return;
    }
  }
}

void readMainCore() {
  while (Serial8.available()) {

    // Main 傳送停止命令
    if (Serial8.peek() == CMD_STOP) {
      Serial8.read();

      vx = 0;
      vy = 0;
      aim_offset = 0;

      subState = SUB_IDLE;
      MotorStop();

      // 清除 STOP 前排隊的舊移動封包。
      // 避免封包頭 0xAA 被誤認為
      // CMD_ATTACK。
      while (Serial8.available()) {
        Serial8.read();
      }

      return;
    }

    if (Serial8.available() < 7) {
      return;
    }

    if (Serial8.read() != 0xAA) {
      continue;
    }

    if (Serial8.read() != 0xAA) {
      continue;
    }

    uint8_t buffer[7];

    buffer[0] = 0xAA;
    buffer[1] = 0xAA;

    for (int i = 2; i < 7; i++) {
      buffer[i] =
          Serial8.read();
    }

    if (buffer[6] != 0xEE) {
      continue;
    }

    uint8_t sum =
        buffer[2] +
        buffer[3] +
        buffer[4];

    if (sum != buffer[5]) {
      continue;
    }

    vx =
        (int8_t)buffer[2];

    vy =
        (int8_t)buffer[3];

    aim_offset =
        (int8_t)buffer[4];

    return;
  }
}

void setup() {
  delay(3000);

  Robot_Init();
  Serial2.begin(115200);

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  pinMode(M1, INPUT);
  pinMode(M2, INPUT);

  EEPROM.begin();
  EEPROM.get(0, avg_ls);

  MotorStop();
}

void loop() {
  if (subState != SUB_ATTACK) {
    readCommand();
    MotorStop();
    return;
  }

  readBNO085Yaw();
  readMainCore();

  // 收到 CMD_STOP 後，
  // readMainCore 會把狀態改成 IDLE。
  if (subState != SUB_ATTACK) {
    MotorStop();
    return;
  }

  fast_update_line_sensor();

  bool onLine =
      moveBackInBounds();

  if (fabs(gyroData.pitch) > 15) {
    finalVx = 0;
    finalVy = 0;

    control.robot_heading = 90;

    MotorStop();

    Serial.println(
        "ROBOT PICKED UP - ALL STATES RESET"
    );

    return;
  }
  
  if (onLine) {
    //Serial.print("line");
    Vector_Motion(
        lineVx,
        lineVy,
        0,
        true
    );
  } 
    
  else {
    bool reset_heading =
        (aim_offset == 0);
    /*
    Serial.print("vx= ");
    Serial.print(vx);

    Serial.print(" vy= ");
    Serial.print(vy);

    Serial.print(" aim= ");
    Serial.println(aim_offset);
    */
   float heading_kp =
    (aim_offset != 0)
    ? 1.5f    // 持球向左右轉頭
    : 0.7f;   // 平常保持90度、回正
    FC_Vector_Motion(
        vx,
        vy,
        90+aim_offset,
        heading_kp
    );
  }
  /*
  Serial.print(
      " vx"
  );
  Serial.print(
      vx
  );
  Serial.print(
      " vy"
  );
  Serial.print(
     vy

  );
  Serial.print(
      " aim_offset"
  );
  Serial.println(
      aim_offset
  );*/
}