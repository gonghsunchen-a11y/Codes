/*#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

enum SubState { SUB_IDLE, SUB_RUN };
SubState subState = SUB_IDLE;

enum Phase {
    PHASE_PUSH_FIRST,
    PHASE_ORBIT,
    PHASE_ATTACK
};
Phase phase = PHASE_PUSH_FIRST;

int16_t  s_ball_angle = 0;
int16_t  s_ball_dist  = 0;
bool     s_ball_valid = false;
int16_t  s_goal_x     = 0;
bool     s_goal_valid = false;
int16_t  s_us_f = 0, s_us_b = 0, s_us_l = 0, s_us_r = 0;

unsigned long phaseTimer = 0;

float final_omega  = 0;
float d_last_error = 0;
unsigned long last_time = 0;

void readMainPacket() {
    static uint8_t buffer[20];
    static int index = 0;

    while (Serial8.available() > 0) {
        uint8_t b = Serial8.read();

        if (subState == SUB_IDLE) {
            if (b == 0xBB) {
                subState     = SUB_RUN;
                phase        = PHASE_PUSH_FIRST;
                phaseTimer   = millis();
                index        = 0;
                Serial.println("START");
            }
            continue;
        }

        if (index == 0 || index == 1) {
            if (b == 0xAA) buffer[index++] = b;
            else index = 0;
            continue;
        }
        buffer[index++] = b;
        if (index == 20) {
            index = 0;
            if (buffer[19] != 0xEE) continue;
            uint8_t sum = 0;
            for (int i = 2; i <= 17; i++) sum += buffer[i];
            if (sum != buffer[18]) continue;

            s_ball_angle = (int16_t)((buffer[3] << 8) | buffer[2]);
            s_ball_dist  = (int16_t)((buffer[5] << 8) | buffer[4]);
            s_ball_valid = buffer[6] == 0xFF;
            s_goal_x     = (int16_t)((buffer[8] << 8) | buffer[7]);
            s_goal_valid = buffer[9] == 0xFF;
            s_us_f       = (int16_t)((buffer[11] << 8) | buffer[10]);
            s_us_b       = (int16_t)((buffer[13] << 8) | buffer[12]);
            s_us_l       = (int16_t)((buffer[15] << 8) | buffer[14]);
            s_us_r       = (int16_t)((buffer[17] << 8) | buffer[16]);
        }
    }
}

void runAttack() {
    if (s_goal_valid) {
        int error = s_goal_x - 130;
        unsigned long now = millis();
        float dt = (now - last_time) / 1000.0f;
        float d_error = 0;
        if (dt > 0 && last_time != 0)
            d_error = (error - d_last_error) / dt;
        d_last_error = error;
        last_time    = now;
        final_omega  = constrain(0.02f * error + 0.03f * d_error, -10.0f, 10.0f);
        if (abs(error) < 40) final_omega = 0;
    } else {
        final_omega  = 0;
        d_last_error = 0;
        last_time    = 0;
    }

    float Vx = 0, Vy = 0;

    if (s_ball_valid) {
        float moving_degree = s_ball_angle;
        float ballspeed = constrain(map(s_ball_dist, 25, 55, 30, 60), 30, 60);

        if (s_ball_dist < 55) {
            if (s_ball_angle >= 84 && s_ball_angle <= 98) {
                ballspeed     = 40;
                moving_degree = 90;
            } else if (s_ball_angle > 98 && s_ball_angle < 270) {
                float offsetRatio  = constrain(exp(-1.5 * (s_ball_dist - 50)), 0.0, 1.0);
                float smoothWeight = constrain(fabs(s_ball_angle - 90) / 30.0f, 0.0f, 1.0f);
                moving_degree = s_ball_angle + 80 * offsetRatio * smoothWeight;
            } else {
                float offsetRatio  = constrain(exp(-1.5 * (s_ball_dist - 50)), 0.0, 1.0);
                float smoothWeight = constrain(fabs(s_ball_angle - 90) / 30.0f, 0.0f, 1.0f);
                moving_degree = s_ball_angle - 80 * offsetRatio * smoothWeight;
            }
        }

        if (s_ball_dist <= 29 && s_ball_angle >= 80 && s_ball_angle <= 100) {
            moving_degree = 90;
            ballspeed     = 80;
        }

        moving_degree = fmod(moving_degree + 360.0f, 360.0f);
        Vx = round(ballspeed * cos(moving_degree * DtoR_const));
        Vy = round(ballspeed * sin(moving_degree * DtoR_const));

        float vxWeight = constrain(fabs(s_ball_angle - 90) / 30.0f, 0.0f, 1.0f);
        Vx = round(Vx * vxWeight * 0.65);

        if (s_us_r <= 16) { if (Vx > 0) Vx = 0; }
        else if (s_us_r <= 20) { if (Vx > 0) Vx *= 0.5; }
        else if (s_us_r <= 24) { if (Vx > 0) Vx *= 0.7; }

        if (s_us_l <= 26) { if (Vx < 0) Vx = 0; }
        else if (s_us_l <= 30) { if (Vx < 0) Vx *= 0.5; }
        else if (s_us_l <= 34) { if (Vx < 0) Vx *= 0.7; }

        if (s_us_f <= 25) { if (Vy > 0) Vy = 0; }
        else if (s_us_f <= 27) { if (Vy > 0) Vy *= 0.6; }
        else if (s_us_f <= 29) { if (Vy > 0) Vy *= 0.8; }

        if (s_us_b <= 25) { if (Vy < 0) Vy = 0; }
        else if (s_us_b <= 27) { if (Vy < 0) Vy *= 0.6; }
        else if (s_us_b <= 29) { if (Vy < 0) Vy *= 0.8; }
    }

    if (s_goal_valid)
        Vector_Motion(Vx, Vy, -final_omega / 2000.0f, false, true);
    else
        Vector_Motion(Vx, Vy, 0, true, true);
}

void runPhases() {
    float Vx = 0, Vy = 0;

    switch (phase) {

        // ── 1. 推球 2 秒 ────────────────────────────────────
        case PHASE_PUSH_FIRST:
        {
            if (s_ball_valid) {
                float angleErr = s_ball_angle - 90.0f;
                if (angleErr >  180) angleErr -= 360;
                if (angleErr < -180) angleErr += 360;
                Vx = constrain(angleErr * 0.8f, -30, 30);
            }
            Vy = 20;

            if (millis() - phaseTimer > 2000) {
                phase      = PHASE_ORBIT;
                phaseTimer = millis();  // 重設計時器給ORBIT用
                Serial.println("→ ORBIT");
            }

            Vector_Motion(Vx, Vy, 0, true, true);
            break;
        }

        // ── 2. 繞球一圈（計時3秒）+ 球在正前方才切 ──────────
                  case PHASE_ORBIT:
        {
            if (!s_ball_valid) {
                Vector_Motion(0, 0, 0, true, false);
                break;
            }

            // 切線方向（順時針繞球）
            float tangent = fmod(s_ball_angle + 90.0f, 360.0f);
            float Vx = 50 * cos(tangent * DtoR_const);
            float Vy = 50 * sin(tangent * DtoR_const);

            Serial.printf("ORBIT t=%lu angle=%d tangent=%.0f\n",
                          millis() - phaseTimer, s_ball_angle, tangent);

            if (millis() - phaseTimer > 5000) {
                final_omega  = 0;
                d_last_error = 0;
                last_time    = 0;
                phase        = PHASE_ATTACK;
                Serial.println("→ ATTACK");
            }

            Vector_Motion(Vx, Vy, 0, true, true);
            break;
        }

        // ── 3. 進攻 ─────────────────────────────────────────
        case PHASE_ATTACK:
            runAttack();
            break;
    }
}

void setup() {
    Robot_Init();
    Serial2.begin(115200);
    Serial.println("testsub ready");
}

void loop() {
    readMainPacket();
    readBNO085Yaw();

    if (subState == SUB_IDLE) return;

    runPhases();
}*/
/*
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

enum SubState { SUB_IDLE, SUB_RUN };
SubState subState = SUB_IDLE;

enum Phase { PHASE_FORWARD, PHASE_BACKWARD, PHASE_ATTACK };
Phase phase = PHASE_FORWARD;

int16_t  s_ball_angle = 0;
int16_t  s_ball_dist  = 0;
bool     s_ball_valid = false;
int16_t  s_goal_x     = 0;
bool     s_goal_valid = false;
int16_t  s_us_f = 0, s_us_b = 0, s_us_l = 0, s_us_r = 0;

unsigned long phaseTimer = 0;
float final_omega  = 0;
float d_last_error = 0;
unsigned long last_time = 0;

void readMainPacket() {
    static uint8_t buffer[20];
    static int index = 0;

    while (Serial8.available() > 0) {
        uint8_t b = Serial8.read();

        if (subState == SUB_IDLE) {
            if (b == 0xBB) {
                subState   = SUB_RUN;
                phase      = PHASE_FORWARD;
                phaseTimer = millis();
                index      = 0;
                Serial.println("START");
            }
            continue;
        }

        if (index == 0 || index == 1) {
            if (b == 0xAA) buffer[index++] = b;
            else index = 0;
            continue;
        }
        buffer[index++] = b;
        if (index == 20) {
            index = 0;
            if (buffer[19] != 0xEE) continue;
            uint8_t sum = 0;
            for (int i = 2; i <= 17; i++) sum += buffer[i];
            if (sum != buffer[18]) continue;

            s_ball_angle = (int16_t)((buffer[3] << 8) | buffer[2]);
            s_ball_dist  = (int16_t)((buffer[5] << 8) | buffer[4]);
            s_ball_valid = buffer[6] == 0xFF;
            s_goal_x     = (int16_t)((buffer[8] << 8) | buffer[7]);
            s_goal_valid = buffer[9] == 0xFF;
            s_us_f       = (int16_t)((buffer[11] << 8) | buffer[10]);
            s_us_b       = (int16_t)((buffer[13] << 8) | buffer[12]);
            s_us_l       = (int16_t)((buffer[15] << 8) | buffer[14]);
            s_us_r       = (int16_t)((buffer[17] << 8) | buffer[16]);
        }
    }
}

void runAttack() {
    if (s_goal_valid) {
        int error = s_goal_x - 130;
        unsigned long now = millis();
        float dt = (now - last_time) / 1000.0f;
        float d_error = 0;
        if (dt > 0 && last_time != 0)
            d_error = (error - d_last_error) / dt;
        d_last_error = error;
        last_time    = now;
        final_omega  = constrain(0.02f * error + 0.03f * d_error, -10.0f, 10.0f);
        if (abs(error) < 40) final_omega = 0;
    } else {
        final_omega  = 0;
        d_last_error = 0;
        last_time    = 0;
    }

    float Vx = 0, Vy = 0;

    if (s_ball_valid) {
        float moving_degree = s_ball_angle;

        if (s_ball_dist < 55) {
            if (s_ball_angle >= 84 && s_ball_angle <= 98) {
                moving_degree = 90;
            } else if (s_ball_angle > 98 && s_ball_angle < 270) {
                float offsetRatio  = constrain(exp(-1.5 * (s_ball_dist - 50)), 0.0, 1.0);
                float smoothWeight = constrain(fabs(s_ball_angle - 90) / 30.0f, 0.0f, 1.0f);
                moving_degree = s_ball_angle + 80 * offsetRatio * smoothWeight;
            } else {
                float offsetRatio  = constrain(exp(-1.5 * (s_ball_dist - 50)), 0.0, 1.0);
                float smoothWeight = constrain(fabs(s_ball_angle - 90) / 30.0f, 0.0f, 1.0f);
                moving_degree = s_ball_angle - 80 * offsetRatio;
            }
        }

        moving_degree = fmod(moving_degree + 360.0f, 360.0f);
        Vx = round(50 * cos(moving_degree * DtoR_const));
        Vy = round(50 * sin(moving_degree * DtoR_const));

        float vxWeight = constrain(fabs(s_ball_angle - 90) / 30.0f, 0.0f, 1.0f);
        Vx = round(Vx * vxWeight * 0.65);

        if (s_us_r <= 16) { if (Vx > 0) Vx = 0; }
        else if (s_us_r <= 20) { if (Vx > 0) Vx *= 0.5; }
        else if (s_us_r <= 24) { if (Vx > 0) Vx *= 0.7; }

        if (s_us_l <= 26) { if (Vx < 0) Vx = 0; }
        else if (s_us_l <= 30) { if (Vx < 0) Vx *= 0.5; }
        else if (s_us_l <= 34) { if (Vx < 0) Vx *= 0.7; }

        if (s_us_f <= 25) { if (Vy > 0) Vy = 0; }
        else if (s_us_f <= 27) { if (Vy > 0) Vy *= 0.6; }
        else if (s_us_f <= 29) { if (Vy > 0) Vy *= 0.8; }

        if (s_us_b <= 25) { if (Vy < 0) Vy = 0; }
        else if (s_us_b <= 27) { if (Vy < 0) Vy *= 0.6; }
        else if (s_us_b <= 29) { if (Vy < 0) Vy *= 0.8; }
    }

    if (s_goal_valid)
        Vector_Motion(Vx, Vy, -final_omega / 2000.0f, false, true);
    else
        Vector_Motion(Vx, Vy, 0, true, true);
}

void runPhases() {
    switch (phase) {

        case PHASE_FORWARD:
            Vector_Motion(0, 20, 0, true, true);
            if (millis() - phaseTimer > 2000) {
                phase      = PHASE_BACKWARD;
                phaseTimer = millis();
                Serial.println("→ BACKWARD");
            }
            break;

        case PHASE_BACKWARD:
            Vector_Motion(0, -20, 0, true, true);
            if (millis() - phaseTimer > 2000) {
                final_omega  = 0;
                d_last_error = 0;
                last_time    = 0;
                phase        = PHASE_ATTACK;
                Serial.println("→ ATTACK");
            }
            break;

        case PHASE_ATTACK:
            runAttack();
            break;
    }
}

void setup() {
    Robot_Init();
    Serial2.begin(115200);
    Serial.println("testsub ready");
}

void loop() {
    readMainPacket();
    readBNO085Yaw();
    if (subState == SUB_IDLE) return;
    runPhases();
}*/
/*
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

enum SubState { SUB_IDLE, SUB_RUN };
SubState subState = SUB_IDLE;

enum Phase {
    PHASE_RIGHT_FORWARD,
    PHASE_RIGHT_BACKWARD,
    PHASE_GO_LEFT,
    PHASE_LEFT_FORWARD,
    PHASE_LEFT_BACKWARD,
    PHASE_STOP
};
Phase phase = PHASE_RIGHT_FORWARD;

unsigned long phaseTimer = 0;

void readMainPacket() {
    static uint8_t buffer[20];
    static int index = 0;

    while (Serial8.available() > 0) {
        uint8_t b = Serial8.read();

        if (subState == SUB_IDLE) {
            if (b == 0xBB) {
                subState   = SUB_RUN;
                phase      = PHASE_RIGHT_FORWARD;
                phaseTimer = millis();
                index      = 0;
                Serial.println("START");
            }
            continue;
        }

        if (index == 0 || index == 1) {
            if (b == 0xAA) buffer[index++] = b;
            else index = 0;
            continue;
        }
        buffer[index++] = b;
        if (index == 20) {
            index = 0;
            if (buffer[19] != 0xEE) continue;
            uint8_t sum = 0;
            for (int i = 2; i <= 17; i++) sum += buffer[i];
            if (sum != buffer[18]) continue;
            // 這題不需要解析感測器資料
        }
    }
}

void runPhases() {
    switch (phase) {

        case PHASE_RIGHT_FORWARD:
            Vector_Motion(0, 20, 0, true, true);
            if (millis() - phaseTimer > 2000) {
                phase      = PHASE_RIGHT_BACKWARD;
                phaseTimer = millis();
                Serial.println("→ RIGHT_BACKWARD");
            }
            break;

        case PHASE_RIGHT_BACKWARD:
            Vector_Motion(0, -20, 0, true, true);
            if (millis() - phaseTimer > 2000) {
                phase      = PHASE_GO_LEFT;
                phaseTimer = millis();
                Serial.println("→ GO_LEFT");
            }
            break;

        case PHASE_GO_LEFT:
            Vector_Motion(-20, 0, 0, true, true);
            if (millis() - phaseTimer > 3000) {  // 往左走3秒到左側
                phase      = PHASE_LEFT_FORWARD;
                phaseTimer = millis();
                Serial.println("→ LEFT_FORWARD");
            }
            break;

        case PHASE_LEFT_FORWARD:
            Vector_Motion(0, 20, 0, true, true);
            if (millis() - phaseTimer > 2000) {
                phase      = PHASE_LEFT_BACKWARD;
                phaseTimer = millis();
                Serial.println("→ LEFT_BACKWARD");
            }
            break;

        case PHASE_LEFT_BACKWARD:
            Vector_Motion(0, -20, 0, true, true);
            if (millis() - phaseTimer > 2000) {
                phase = PHASE_STOP;
                Serial.println("→ STOP");
            }
            break;

        case PHASE_STOP:
            MotorStop();
            break;
    }
}

void setup() {
    Robot_Init();
    Serial2.begin(115200);
    Serial.println("testsub ready");
}

void loop() {
    readMainPacket();
    readBNO085Yaw();
    if (subState == SUB_IDLE) return;
    runPhases();
}*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

enum SubState { SUB_IDLE, SUB_RUN };
SubState subState = SUB_IDLE;

enum Phase {
    PHASE_FORWARD,  // 直走追球到前牆
    PHASE_LEFT,     // 左走追球
    PHASE_STOP
};
Phase phase = PHASE_FORWARD;

int16_t  s_ball_angle = 0;
int16_t  s_ball_dist  = 0;
bool     s_ball_valid = false;
int16_t  s_us_f = 0, s_us_b = 0, s_us_l = 0, s_us_r = 0;

unsigned long phaseTimer = 0;

void readMainPacket() {
    static uint8_t buffer[20];
    static int index = 0;

    while (Serial8.available() > 0) {
        uint8_t b = Serial8.read();

        if (subState == SUB_IDLE) {
            if (b == 0xBB) {
                subState   = SUB_RUN;
                phase      = PHASE_FORWARD;
                phaseTimer = millis();
                index      = 0;
                Serial.println("START");
            }
            continue;
        }

        if (index == 0 || index == 1) {
            if (b == 0xAA) buffer[index++] = b;
            else index = 0;
            continue;
        }
        buffer[index++] = b;
        if (index == 20) {
            index = 0;
            if (buffer[19] != 0xEE) continue;
            uint8_t sum = 0;
            for (int i = 2; i <= 17; i++) sum += buffer[i];
            if (sum != buffer[18]) continue;

            s_ball_angle = (int16_t)((buffer[3] << 8) | buffer[2]);
            s_ball_dist  = (int16_t)((buffer[5] << 8) | buffer[4]);
            s_ball_valid = buffer[6] == 0xFF;
            s_us_f       = (int16_t)((buffer[11] << 8) | buffer[10]);
            s_us_b       = (int16_t)((buffer[13] << 8) | buffer[12]);
            s_us_l       = (int16_t)((buffer[15] << 8) | buffer[14]);
            s_us_r       = (int16_t)((buffer[17] << 8) | buffer[16]);
        }
    }
}

void runPhases() {
    float Vx = 0, Vy = 0;

    switch (phase) {

        // ── 直走，球跑掉就追 ────────────────────────────────
             case PHASE_FORWARD:
        {
            if (s_ball_valid) {
                float rad = s_ball_angle * DtoR_const;
                Vx = 15 * cos(rad);
                Vy = 15 * sin(rad);
                if (Vy < 10) Vy = 10;
            } else {
                Vy = 15;
            }
            if (millis() - phaseTimer > 3000) {
                phase      = PHASE_LEFT;
                phaseTimer = millis();
                Serial.println("→ LEFT");
            }
            Vector_Motion(Vx, Vy, 0, true, true);
            break;
        }

        case PHASE_LEFT:
        {
            if (s_ball_valid) {
                float rad = s_ball_angle * DtoR_const;
                Vx = 15 * cos(rad);
                Vy = 15 * sin(rad);
                if (Vx > -10) Vx = -10;
            } else {
                Vx = -15;
            }
            if (millis() - phaseTimer > 3000) {
                phase = PHASE_STOP;
                Serial.println("→ STOP");
            }
            Vector_Motion(Vx, Vy, 0, true, true);
            break;
        }
        case PHASE_STOP:
            MotorStop();
            break;
    }
}

void setup() {
    Robot_Init();
    Serial2.begin(115200);
    Serial.println("testsub ready");
}

void loop() {
    readMainPacket();
    readBNO085Yaw();
    if (subState == SUB_IDLE) return;
    runPhases();
}

/*
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

enum SubState { SUB_IDLE, SUB_RUN };
SubState subState = SUB_IDLE;

// 繞行方向：沿右牆前進 → 沿前牆左移 → 沿左牆後退 → 沿後牆右移 → 回起點
enum Phase {
    PHASE_NORTH,   // 沿右牆往前（+Vy），維持 us_r = 20
    PHASE_WEST,    // 沿前牆往左（-Vx），維持 us_f = 20
    PHASE_SOUTH,   // 沿左牆往後（-Vy），維持 us_l = 20
    PHASE_EAST,    // 沿後牆往右（+Vx），維持 us_b = 20
    PHASE_STOP
};
Phase phase = PHASE_NORTH;

int16_t s_us_f = 0, s_us_b = 0, s_us_l = 0, s_us_r = 0;
unsigned long phaseTimer = 0;

#define TARGET_DIST  20     // 維持距離 cm
#define BASE_SPEED   25     // 行進速度
#define KP           1.2f   // P 增益：距離誤差 → 側向修正

// 換邊條件：靠近前方牆壁 20cm 時切換 phase
#define CORNER_DIST  22

void readMainPacket() {
    static uint8_t buffer[20];
    static int index = 0;

    while (Serial8.available() > 0) {
        uint8_t b = Serial8.read();

        if (subState == SUB_IDLE) {
            if (b == 0xBB) {
                subState   = SUB_RUN;
                phase      = PHASE_NORTH;
                phaseTimer = millis();
                index      = 0;
                Serial.println("START");
            }
            continue;
        }

        if (index == 0 || index == 1) {
            if (b == 0xAA) buffer[index++] = b;
            else index = 0;
            continue;
        }
        buffer[index++] = b;
        if (index == 20) {
            index = 0;
            if (buffer[19] != 0xEE) continue;
            uint8_t sum = 0;
            for (int i = 2; i <= 17; i++) sum += buffer[i];
            if (sum != buffer[18]) continue;

            s_us_f = (int16_t)((buffer[11] << 8) | buffer[10]);
            s_us_b = (int16_t)((buffer[13] << 8) | buffer[12]);
            s_us_l = (int16_t)((buffer[15] << 8) | buffer[14]);
            s_us_r = (int16_t)((buffer[17] << 8) | buffer[16]);
        }
    }
}

void runPhases() {
    float Vx = 0, Vy = 0;
    float err = 0;

    switch (phase) {

        // ── 往前走，貼著右牆 ─────────────────────────────
        case PHASE_NORTH:
            Vy  = BASE_SPEED;
            err = s_us_r - TARGET_DIST;          // 右牆誤差
            Vx  = constrain(KP * err, -15.0f, 15.0f);
            if (s_us_f <= CORNER_DIST) {         // 碰到前牆 → 轉彎
                phase = PHASE_WEST;
                Serial.println("-> WEST");
            }
            break;

        // ── 往左走，貼著前牆 ─────────────────────────────
        case PHASE_WEST:
            Vx  = -BASE_SPEED;
            err = s_us_f - TARGET_DIST;          // 前牆誤差
            Vy  = constrain(KP * err, -15.0f, 15.0f);
            if (s_us_l <= CORNER_DIST) {         // 碰到左牆 → 轉彎
                phase = PHASE_SOUTH;
                Serial.println("-> SOUTH");
            }
            break;

        // ── 往後走，貼著左牆 ─────────────────────────────
        case PHASE_SOUTH:
            Vy  = -BASE_SPEED;
            err = s_us_l - TARGET_DIST;          // 左牆誤差
            Vx  = constrain(-KP * err, -15.0f, 15.0f);  // 注意方向反轉
            if (s_us_b <= CORNER_DIST) {         // 碰到後牆 → 轉彎
                phase = PHASE_EAST;
                Serial.println("-> EAST");
            }
            break;

        // ── 往右走，貼著後牆 ─────────────────────────────
        case PHASE_EAST:
            Vx  = BASE_SPEED;
            err = s_us_b - TARGET_DIST;          // 後牆誤差
            Vy  = constrain(-KP * err, -15.0f, 15.0f);  // 注意方向反轉
            if (s_us_r <= CORNER_DIST) {         // 回到起點右牆 → 停止（或重新循環）
                phase = PHASE_NORTH;             // 改成 PHASE_STOP 若只繞一圈
                Serial.println("-> NORTH (loop)");
            }
            break;

        case PHASE_STOP:
            MotorStop();
            return;
    }

    Serial.printf("phase=%d Vx=%.1f Vy=%.1f | f=%d b=%d l=%d r=%d\n",
                  phase, Vx, Vy, s_us_f, s_us_b, s_us_l, s_us_r);

    Vector_Motion(Vx, Vy, 0, true, false);
}

void setup() {
    Robot_Init();
    Serial2.begin(115200);
    Serial.println("rect follow ready");
}

void loop() {
    readMainPacket();
    readBNO085Yaw();
    if (subState == SUB_IDLE) return;
    runPhases();
}*///繞球場
/*
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

enum Phase { PHASE_FORWARD, PHASE_BACKWARD, PHASE_STOP };
Phase phase = PHASE_FORWARD;

unsigned long phaseTimer = 0;

void setup() {
    Robot_Init();
    Serial2.begin(115200);
    phaseTimer = millis();
    Serial.println("START FORWARD");
}

void loop() {
    readBNO085Yaw();

    switch (phase) {

        case PHASE_FORWARD:
            Vector_Motion(0, 20, 0, true, false);
            if (millis() - phaseTimer > 2000) {
                phase      = PHASE_BACKWARD;
                phaseTimer = millis();
                Serial.println("→ BACKWARD");
            }
            break;

        case PHASE_BACKWARD:
            Vector_Motion(0, -20, 0, true, false);
            if (millis() - phaseTimer > 2000) {
                phase = PHASE_STOP;
                Serial.println("→ STOP");
            }
            break;

        case PHASE_STOP:
            MotorStop();
            break;
    }
}*/