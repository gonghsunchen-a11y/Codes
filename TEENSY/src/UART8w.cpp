#include <Arduino.h>
#include <Robot.h>

void printPacket(const uint8_t *packet, uint8_t len){
  for(uint8_t i = 0; i < len; i++){
    if(packet[i] < 0x10) Serial.print("0");
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void sendMovePacket(int8_t vx, int8_t vy, int8_t aim_offset){
  uint8_t packet[7];

  packet[0] = 0xAA;
  packet[1] = 0xAA;
  packet[2] = (uint8_t)vx;
  packet[3] = (uint8_t)vy;
  packet[4] = (uint8_t)aim_offset;
  packet[5] = packet[2] + packet[3] + packet[4];
  packet[6] = 0xEE;

  Serial8.write(packet, 7);

  Serial.print("TX ");
  printPacket(packet, 7);
}

void setup(){
  Robot_Init();
}

void loop(){
  static uint32_t lastSend = 0;
  static int8_t vx = -40;
  static int8_t vy = 80;
  static int8_t aim = -20;


  sendMovePacket(vx, vy, aim);

  vx += 10;
  if(vx > 40) vx = -40;

  aim += 5;
  if(aim > 20) aim = -20;
}
