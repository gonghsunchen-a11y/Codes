#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>
#include <EEPROM.h>

#define M1 A0
#define M2 A1

#define s0 A2
#define s1 A3
#define s2 A4
#define s3 A5
#define LS_count 32

struct LineData{uint32_t state = 0xFFFFFFFF;} lineData;
uint8_t goal_valid = 0x00;
bool readMainCore();
int readMux(int ch, int sigPin);
void line_calibrate();
void linesensor_update();
bool moveBackInBounds();

//ball
float vx, vy, omega;float ballDeg;int ballDist;

//line
uint16_t max_ls[LS_count];
uint16_t avg_ls[LS_count];
uint16_t min_ls[LS_count];
//SPEED
float lineVx = 0;
float lineVy = 0;

float init_lineDegree = -1;
float diff = 0;
bool emergency = false;
bool start = false;
bool overhalf = false;
bool first_detect = false;
float lastLineAngle = -1;       // 紀錄最後一次撞線的角度
uint32_t offLineTimer = 0;      // 離開線後的時間計時
const uint32_t MEMORY_TIME = 800;
uint32_t lastPacketTime = 0;

float finalVx;
float finalVy;


int readMux(int ch, int sigPin) {

  digitalWrite(s0, (ch >> 0) & 1);
  digitalWrite(s1, (ch >> 1) & 1);
  digitalWrite(s2, (ch >> 2) & 1);
  digitalWrite(s3, (ch >> 3) & 1);
  if(sigPin == 1)return analogRead(M1);
  if(sigPin == 2)return analogRead(M2);
}

//量線
void line_calibrate(){
  for(int i=0; i<LS_count; i++){
    max_ls[i] = 0;
    min_ls[i] = 4095;
  }
  while(1){
    if(Serial8.available()){
    uint8_t header = Serial8.peek();
      if (header == 0xFF) {
        if (Serial8.available() >= 3) {
          Serial8.read(); // 讀掉第一個 0xFF
          Serial8.read(); // 讀掉第二個 0xFF
          uint8_t cmd = Serial8.read();
          
          if(cmd == 0x02){
            for(uint8_t i = 0; i < LS_count; i++){
              Serial.print(" min ");Serial.print(i);Serial.print(" = ");Serial.print(min_ls[i]);
              Serial.print(" max ");Serial.print(i);Serial.print(" = ");Serial.print(max_ls[i]);
              Serial.print(" avg ");Serial.print(i);Serial.print(" = ");Serial.print(avg_ls[i]);
              Serial.println("");
            }
            break;
          }
        } 
      }
    }
    for(uint8_t i = 0; i < LS_count; i++){
    uint16_t reading = readMux(i % 16, (i < 16) ? 1 : 2);

    if(reading > max_ls[i]) max_ls[i] = reading;
    if(reading < min_ls[i]) min_ls[i] = reading;
    } 
  }

  for(uint8_t i = 0; i < LS_count; i++){
      avg_ls[i] = (max_ls[i] + min_ls[i]) / 2;
  }
  EEPROM.put(0, avg_ls);
  Serial8.write(0xFF);
  Serial8.write(0xFF);
  Serial8.write(0x03);
}

//更新
void linesensor_update(){
  lineData.state = 0xFFFFFFFF;
  
  for (uint8_t i = 0; i < LS_count; i++) {
    uint16_t reading = readMux(i % 16, (i < 16) ? 1 : 2);;
    
    if (reading < avg_ls[i]) {
      lineData.state &= ~(1UL << i); 
      //Serial.printf("%d,%d",i,reading);
      //Serial.print(" avg ");Serial.print(i);Serial.print(" = ");Serial.print(avg_ls[i]);
      //Serial.println();
    }
  }
  /*
  for (int i = LS_count - 1; i >= 0; i--) {
    uint8_t bit = (lineData.state >> i) & 1;
    Serial.print(bit);

    if (i % 4 == 0 && i != 0) {
      Serial.print(" "); 
    }
  }
  Serial.println(" ");
  delay(50);
  */
}
bool moveBackInBounds(){
  //-----LINE SENSOR-----
  float sumX = 0.0f, sumY = 0.0f;
  int count = 0;
  bool linedetected = false;
  for(int i = 0; i < LS_count; i++){
    if(bitRead(lineData.state, i) == 0){
      //if(i==0){continue;}
      
      //Serial.printf("read%d", i);
      
      float deg = linesensorDegreelist[i];
      sumX += cos(deg * DtoR_const);
      sumY += sin(deg * DtoR_const);
      count++;
      linedetected = true;
    }
  }

  // B : 反彈

  if(linedetected && count >= 1){
    float lineDegree = atan2(sumY, sumX) * RtoD_const;
    if (lineDegree < 0){lineDegree += 360;} 
    
    //Serial.print("degree=");Serial.println(lineDegree);

    if (!first_detect){
      init_lineDegree = lineDegree;
      first_detect = true;

      Serial.println("LINE DETECTED !!!");
      Serial.print("initlineDegree =");Serial.println(init_lineDegree);
    }

    diff = fabs(lineDegree - init_lineDegree);
    if(diff > 180){diff = 360 - diff;}
    
    //Serial.print("diff =");Serial.println(diff);


    //-----BACK TO FIELD-----
    float finalDegree;
    if(diff > EMERGENCY_THRESHOLD){
      overhalf = true;
      finalDegree = fmod(init_lineDegree + 180.0f, 360.0f);
    }
    else{
      overhalf = false;
      finalDegree = fmod(lineDegree + 180.0f, 360.0f);
    }
    Serial.print("finalDegree =");Serial.println(finalDegree);
        
    lineVx = 40.0f *cos(finalDegree * DtoR_const);
    lineVy = 40.0f *sin(finalDegree * DtoR_const);
    
    lastLineAngle = lineDegree; // 更新最後的線角度
    offLineTimer = millis();
    return true;  
  }
  else{
    init_lineDegree = -1;
    diff = 0;
    lineVx = 0;
    lineVy = 0;
    overhalf = false;
    first_detect = false;
    return false;
  }
}

void readMainPacket(){
    static uint8_t buffer[11];
    static int index = 0;
    while(Serial8.available() > 0){
        uint8_t b = Serial8.read();
        if(index == 0 || index == 1){
            if(b == 0xAA) buffer[index++] = b;
            else index = 0;
            continue;
        }
        buffer[index++] = b;
        if(index == 11){
            index = 0;
            if(buffer[10] != 0xEE) continue;
            uint8_t sum = 0;
            for(int i = 2; i <= 8; i++) sum += buffer[i];
            if(sum != buffer[9]) continue;
            vx         = (int16_t)((buffer[3] << 8) | buffer[2]);
            vy         = (int16_t)((buffer[5] << 8) | buffer[4]);
            omega      = (int16_t)((buffer[7] << 8) | buffer[6]);
            goal_valid =  buffer[8];
        }
    }
}
void setup(){
  Robot_Init();
  Serial2.begin(115200);

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  pinMode(M1, INPUT_PULLDOWN);
  pinMode(M2, INPUT_PULLDOWN);
  
  EEPROM.begin();
  EEPROM.get(0, avg_ls);
  // 初始化 max / min
  delay(1000);
  Vector_Motion(0,25,0,1,0);
  delay(1500);
}

void loop(){
  readBNO085Yaw();
  readMainPacket();
      finalVx = vx;
      finalVy = vy;
  //finalVx = vx;
  //finalVy = vy;
  Vector_Motion(finalVx,finalVy,omega,0,0);
  Serial.print("vx= ");Serial.println(finalVx);
  Serial.print("vy= ");Serial.println(finalVy);

}