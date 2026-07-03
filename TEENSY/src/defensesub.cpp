#include <Arduino.h>
#include <Robot.h>

#define CMD_ATTACK 0xAA
#define CMD_STOP   0xBB

enum SubState {
  SUB_IDLE,
  SUB_ATTACK
};
float vx;
float vy;

int aim_offset;
SubState subState = SUB_IDLE;

uint8_t receive_packet[7];
uint8_t receive_index = 0;

int8_t command_vx = 0;
int8_t command_vy = 0;

bool packet_received = false;
uint32_t last_packet_time = 0;

const uint32_t PACKET_TIMEOUT_MS = 200;
const float TARGET_HEADING = 135.0f;

void stopRobot() {
  command_vx = 0;
  command_vy = 0;
  packet_received = false;

  MotorStop();
}


void readMainCore(){
  while(Serial8.available()){
    if(Serial8.available() < 7) return;

    if(Serial8.read() != 0xAA) continue;
    if(Serial8.read() != 0xAA) continue;

    uint8_t buffer[7];
    buffer[0] = 0xAA;
    buffer[1] = 0xAA;

    for(int i = 2; i < 7; i++){
      buffer[i] = Serial8.read();
    }

    if(buffer[6] != 0xEE) continue;

    uint8_t sum = buffer[2] + buffer[3] + buffer[4];
    if(sum != buffer[5]) continue;

    vx = (int8_t)buffer[2];
    vy = (int8_t)buffer[3];
    aim_offset = (int8_t)buffer[4];

    return;
  }
}

void setup() {
  //delay(3000);
  Serial2.begin(115200);
  Robot_Init();

  command_vx = 0;
  command_vy = 0;

  stopRobot();

  Serial.println("SUB READY");
}

void loop() {

  readBNO085Yaw();
  readMainCore();
  Serial.print("head= ");Serial.print(gyroData.heading);
  Serial.print(" offset= ");Serial.print(aim_offset);
  Serial.print(" Vy= ");Serial.println(vy);
  FC_Vector_Motion(vx,vy,90+aim_offset);
}