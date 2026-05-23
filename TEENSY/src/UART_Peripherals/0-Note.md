// Bit 位址選通：把多工器的通道編號 ch 轉成 s0~s3 的控制訊號

digitalWrite(s0, (ch >> 0) & 1); // bit0 → s0，右移 0 位，取最低位
digitalWrite(s1, (ch >> 1) & 1); // bit1 → s1，右移 1 位，取下一位
digitalWrite(s2, (ch >> 2) & 1); // bit2 → s2
digitalWrite(s3, (ch >> 3) & 1); // bit3 → s3，最高位

// 範例：ch = 3 → 二進位 0011
// s0 = 1 (bit0)
// s1 = 1 (bit1)
// s2 = 0 (bit2)
// s3 = 0 (bit3)
// 最後對應 (s3, s2, s1, s0) = 0 0 1 1

// 修正 sensor 位置：30號和31號 sensor 物理裝反，所以交換 index
int mapIndex(int i){
    if(i == 30) return 31; // sensor 30 → map 到 31
    if(i == 31) return 30; // sensor 31 → map 到 30
    return i;              // 其他不變
}

// 用陣列一次定義所有 sensor 的硬體對應
const uint8_t mapTable[32] = {
    0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,
    16,17,18,19,20,21,22,23,24,25,26,27,28,29,31,30
};

// 循環讀 32 顆 sensor
for(int i = 0; i < 32; i++){ 
    ls_data[i] = readMux(mapTable[i], (i < 16) ? M1 : M2); 
    // 三元運算符 (condition ? value_if_true : value_if_false)
    // i < 16 → 讀 M1，多工器1
    // i >=16 → 讀 M2，多工器2
}
/----------------------------------
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define LAP_SPEED    45
#define WALL_TARGET  30
#define WALL_KP      1.2f
#define WALL_MAX     25
#define CORNER_DIST  35

int   lapSeg  = 0;
bool  lapDone = false;

bool btnPressed(int pin) {
    static unsigned long last[40] = {0};
    if (digitalRead(pin) == LOW && millis() - last[pin] > 200) {
        last[pin] = millis();
        return true;
    }
    return false;
}

void sendPacket(float Vx, float Vy, float omega_raw, uint8_t goal) {
    int16_t vx_i = (int16_t)Vx;
    int16_t vy_i = (int16_t)Vy;
    int16_t om_i = (int16_t)omega_raw;

    uint8_t packet[11];
    packet[0] = 0xAA;
    packet[1] = 0xAA;
    packet[2] = vx_i & 0xFF;
    packet[3] = (vx_i >> 8) & 0xFF;
    packet[4] = vy_i & 0xFF;
    packet[5] = (vy_i >> 8) & 0xFF;
    packet[6] = om_i & 0xFF;
    packet[7] = (om_i >> 8) & 0xFF;
    packet[8] = goal;

    uint8_t sum = 0;
    for (int i = 2; i <= 8; i++) sum += packet[i];
    packet[9]  = sum;
    packet[10] = 0xEE;

    Serial8.write(packet, 11);
}

void setup() {
    Robot_Init();
    Serial.println("testmain ready - BTN_UP to start lap");
}

void loop() {
    readBNO085Yaw();
    readussensor();

    // BTN_UP 啟動
    if (btnPressed(BTN_UP)) {
        Serial8.write(0xAA);  // 通知 sub 進入 ATTACK
        lapSeg  = 0;
        lapDone = false;
        Serial.println("LAP START");
    }

    float Vx = 0, Vy = 0;

    if (!lapDone) {
        switch (lapSeg) {
            case 0: {
                float err = usData.dist_f - WALL_TARGET;
                Vy = constrain(WALL_KP * err, -WALL_MAX, WALL_MAX);
                Vx = LAP_SPEED;
                if (usData.dist_r < CORNER_DIST) { lapSeg = 1; Serial.println("CORNER 0→1"); }
                break;
            }
            case 1: {
                float err = usData.dist_r - WALL_TARGET;
                Vx = constrain(WALL_KP * err, -WALL_MAX, WALL_MAX);
                Vy = -LAP_SPEED;
                if (usData.dist_b < CORNER_DIST) { lapSeg = 2; Serial.println("CORNER 1→2"); }
                break;
            }
            case 2: {
                float err = usData.dist_b - WALL_TARGET;
                Vy = constrain(-WALL_KP * err, -WALL_MAX, WALL_MAX);
                Vx = -LAP_SPEED;
                if (usData.dist_l < CORNER_DIST) { lapSeg = 3; Serial.println("CORNER 2→3"); }
                break;
            }
            case 3: {
                float err = usData.dist_l - WALL_TARGET;
                Vx = constrain(-WALL_KP * err, -WALL_MAX, WALL_MAX);
                Vy = LAP_SPEED;
                if (usData.dist_f < CORNER_DIST) { lapSeg = 4; Serial.println("CORNER 3→done"); }
                break;
            }
            default:
                lapDone = true;
                Serial.println("LAP DONE!");
                break;
        }
    }

    sendPacket(Vx, Vy, 0, 0x00);

    Serial.printf("seg=%d f=%d b=%d l=%d r=%d Vx=%.0f Vy=%.0f\n",
        lapSeg, usData.dist_f, usData.dist_b,
        usData.dist_l, usData.dist_r, Vx, Vy);
}
main

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

enum SubState { SUB_IDLE, SUB_ATTACK };
SubState subState = SUB_IDLE;

float vx = 0, vy = 0, omega = 0;
uint8_t goal_valid = 0x00;

void readCommand() {
    while (Serial8.available() > 0) {
        uint8_t cmd = Serial8.read();
        if (cmd == 0xAA) subState = SUB_ATTACK;
    }
}

void readMainPacket() {
    static uint8_t buffer[11];
    static int index = 0;
    while (Serial8.available() > 0) {
        uint8_t b = Serial8.read();
        if (index == 0 || index == 1) {
            if (b == 0xAA) buffer[index++] = b;
            else index = 0;
            continue;
        }
        buffer[index++] = b;
        if (index == 11) {
            index = 0;
            if (buffer[10] != 0xEE) continue;
            uint8_t sum = 0;
            for (int i = 2; i <= 8; i++) sum += buffer[i];
            if (sum != buffer[9]) continue;
            vx         = (int16_t)((buffer[3] << 8) | buffer[2]);
            vy         = (int16_t)((buffer[5] << 8) | buffer[4]);
            omega      = (int16_t)((buffer[7] << 8) | buffer[6]);
            goal_valid =  buffer[8];
        }
    }
}

void setup() {
    Robot_Init();
    Serial2.begin(115200);
    Serial.println("testsub ready");
}

void loop() {
    if (subState != SUB_ATTACK) {
        readCommand();
        return;
    }

    readMainPacket();
    readBNO085Yaw();

    // heading 永遠鎖 90°，omega 忽略
    Vector_Motion(vx, vy, 0, true, 0);
}
sub
/繞長方形
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define ORBIT_RADIUS  50.0f   // 目標距離 cm
#define ORBIT_SPEED   35.0f   // 切線速度
#define ORBIT_KR       0.8f   // 徑向增益
#define FACE_KP        0.15f  // 面球旋轉增益

bool running = false;

bool btnPressed(int pin){
    static unsigned long last[40] = {0};
    if(digitalRead(pin) == LOW && millis() - last[pin] > 200){
        last[pin] = millis();
        return true;
    }
    return false;
}

void setup(){
    Robot_Init();
    drawMessage("TEST READY");
}

void loop(){
    if(btnPressed(BTN_UP))   { running = true;  drawMessage("RUN"); }
    if(btnPressed(BTN_ESC))  { running = false; MotorStop(); drawMessage("STOP"); }
    if(!running) return;

    readBNO085Yaw();
    readBallCam();
    //readussensor();

    float Vx = 0, Vy = 0, omega = 0;

    if(ballData.valid){
        int ballAngle = ballData.angle;
        int ballDist  = ballData.dist;

        // 切線（順時針）
        float tang = fmod(ballAngle - 90.0f + 360.0f, 360.0f);
        Vx = ORBIT_SPEED * cos(tang * DtoR_const);
        Vy = ORBIT_SPEED * sin(tang * DtoR_const);

        // 徑向修正
        float radErr = constrain(ORBIT_KR * (ballDist - ORBIT_RADIUS), -30.0f, 30.0f);
        Vx += radErr * cos(ballAngle * DtoR_const);
        Vy += radErr * sin(ballAngle * DtoR_const);

        // 面球旋轉：讓 ballAngle → 90
        omega = (ballAngle - 90.0f) * FACE_KP;
        omega = constrain(omega, -15.0f, 15.0f);

      
    }
    Serial.print("Vx");Serial.println(Vx);
    Serial.print("Vy");Serial.println(Vy);
    Serial.print("omega");Serial.println(omega);
    // 打包送 sub
    int16_t vx_i = (int16_t)Vx;
    int16_t vy_i = (int16_t)Vy;
    int16_t om_i = (int16_t)(omega * 100.0f);

    uint8_t pkt[11];
    pkt[0] = 0xAA; pkt[1] = 0xAA;
    pkt[2] = vx_i & 0xFF;        pkt[3] = (vx_i >> 8) & 0xFF;
    pkt[4] = vy_i & 0xFF;        pkt[5] = (vy_i >> 8) & 0xFF;
    pkt[6] = om_i & 0xFF;        pkt[7] = (om_i >> 8) & 0xFF;
    pkt[8] = 0x00;
    uint8_t sum = 0;
    for(int i = 2; i <= 8; i++) sum += pkt[i];
    pkt[9] = sum; pkt[10] = 0xEE;
    Serial8.write(pkt, 11);
}
main 繞球
//////////////////////////////

