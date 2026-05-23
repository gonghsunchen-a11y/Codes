#include <Arduino.h>
#include <EEPROM.h>
#include <Robot.h>

#define M1 A0
#define M2 A1

#define s0 A2
#define s1 A3
#define s2 A4
#define s3 A5
#define LS_count 32

uint32_t ls_state = 0;

const uint8_t mapTable[32] = {
  0, 1, 2, 3, 4, 5, 6, 7,
  8, 9, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26, 27, 28, 29, 30, 31
};

uint16_t max_ls[LS_count];
uint16_t avg_ls[LS_count];
uint16_t min_ls[LS_count];

int readMux(int ch, int sigPin);

int readMux(int ch, int sigPin) {

  digitalWrite(s0, (ch >> 0) & 1);
  digitalWrite(s1, (ch >> 1) & 1);
  digitalWrite(s2, (ch >> 2) & 1);
  digitalWrite(s3, (ch >> 3) & 1);
  delay(1);
  uint16_t temp = analogRead(sigPin);
  digitalWrite(s0, 0);
  digitalWrite(s1, 0);
  digitalWrite(s2, 0);
  digitalWrite(s3, 0);
  return temp;
}

void setup() {

  Serial.begin(9600);
  
  EEPROM.begin();

  // 初始化 max / min
  for(int i=0;i<LS_count;i++){
    max_ls[i] = 0;
    min_ls[i] = 4095;
  }
  
}

void loop() {
  for(int t=0; t<500; t++){
    for(uint8_t i = 0; i < LS_count; i++){
    uint16_t reading = readMux(i, (i < 16) ? M1 : M2);

    if(reading > max_ls[i]) max_ls[i] = reading;
    if(reading < min_ls[i]) min_ls[i] = reading;
    } 
  }
  
  for(uint8_t i = 0; i < LS_count; i++){
    avg_ls[i] = (max_ls[i] + min_ls[i]) / 2;
  }

  EEPROM.put(0, avg_ls);

  Serial.println("Saved to EEPROM");
  Serial.println("AVG values:");

  // 印出 avg
  for(uint8_t i = 0; i < LS_count; i++){
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(avg_ls[i]);
    delay(5);
  }

  Serial.println("Finished");
}