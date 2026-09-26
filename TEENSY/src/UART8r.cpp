#include <Arduino.h>
#include <Robot.h>

int8_t vx = 0;
int8_t vy = 0;
int8_t aim_offset = 0;

void printPacket(const uint8_t *packet, uint8_t len){
  for(uint8_t i = 0; i < len; i++){
    if(packet[i] < 0x10) Serial.print("0");
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

bool readMovePacket(){
  static uint8_t buffer[7];
  static uint8_t idx = 0;

  while(Serial8.available()){
    uint8_t b = Serial8.read();

    if(idx == 0){
      if(b != 0xAA) continue;
      buffer[idx++] = b;
      continue;
    }

    if(idx == 1){
      if(b != 0xAA){
        idx = 0;
        continue;
      }
      buffer[idx++] = b;
      continue;
    }

    buffer[idx++] = b;

    if(idx >= 7){
      idx = 0;

      if(buffer[6] != 0xEE){
        Serial.print("BAD END ");
        printPacket(buffer, 7);
        return false;
      }

      uint8_t sum = buffer[2] + buffer[3] + buffer[4];
      if(sum != buffer[5]){
        Serial.print("BAD SUM ");
        printPacket(buffer, 7);
        return false;
      }

      vx = (int8_t)buffer[2];
      vy = (int8_t)buffer[3];
      aim_offset = (int8_t)buffer[4];

      Serial.print("RX ");
      printPacket(buffer, 7);
      Serial.print("vx=");
      Serial.print(vx);
      Serial.print(" vy=");
      Serial.print(vy);
      Serial.print(" aim=");
      Serial.println(aim_offset);

      return true;
    }
  }

  return false;
}

void setup(){
  Robot_Init();
}

void loop(){
  readMovePacket();
}
