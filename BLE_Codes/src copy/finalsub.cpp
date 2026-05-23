#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>
#include <EEPROM.h>

enum SubState { SUB_IDLE, SUB_LINECAL, SUB_ATTACK };
SubState subState = SUB_IDLE;
#define M1 A0
#define M2 A1

#define s0 A2
#define s1 A3
#define s2 A4
#define s3 A5
#define LS_count 32

struct LineData{uint32_t state = 0xFFFFFFFF;} lineData;

int readMux(int ch, int sigPin);
void line_calibrate();
void linesensor_update();
bool moveBackInBounds();
void fast_update_line_sensor();

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
uint32_t speed_timer = 0;

float vx, vy, omega;
uint8_t goal_valid = 0x00;

float finalVx,finalVy;

int readMux(int ch, int sigPin) {

  digitalWrite(s0, (ch >> 0) & 1);
  digitalWrite(s1, (ch >> 1) & 1);
  digitalWrite(s2, (ch >> 2) & 1);
  digitalWrite(s3, (ch >> 3) & 1);
  //delayMicroseconds(10);
  if(sigPin == 1)return analogRead(M1);
  if(sigPin == 2)return analogRead(M2);
}

//量線
void line_calibrate(){
  for(int i=0; i<LS_count; i++){
    max_ls[i] = 0;
    min_ls[i] = 4095;
  }
  uint32_t calStart = millis();
  while(1){
    if(millis() - calStart > 30000) break;  // 30s timeout 防止永久卡死

    if(Serial8.available()){
      if(Serial8.read() == 0xEE){
        for(uint8_t i = 0; i < LS_count; i++){
          avg_ls[i] = (max_ls[i] + min_ls[i]) / 2;
          Serial.print(" min ");Serial.print(i);Serial.print(" = ");Serial.print(min_ls[i]);
          Serial.print(" max ");Serial.print(i);Serial.print(" = ");Serial.print(max_ls[i]);
          Serial.print(" avg ");Serial.print(i);Serial.print(" = ");Serial.print(avg_ls[i]);
          Serial.println("");
        }
        break;
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
  Serial8.write(0xDD);  // 回傳確認
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

  /*for (int i = LS_count - 1; i >= 0; i--) {
    uint8_t bit = (lineData.state >> i) & 1;
    Serial.print(bit);

    if (i % 4 == 0 && i != 0) {
      Serial.print(" "); 
    }
  }
  Serial.println(" ");
  delay(50);*/

}
void fast_update_line_sensor(){
  static uint32_t prevRaw = 0xFFFFFFFF;
  uint32_t rawState       = 0xFFFFFFFF;

  for(uint8_t ch = 0; ch < 16; ch++){
    digitalWriteFast(s0, (ch >> 0) & 1);
    digitalWriteFast(s1, (ch >> 1) & 1);
    digitalWriteFast(s2, (ch >> 2) & 1);
    digitalWriteFast(s3, (ch >> 3) & 1);
    //delayMicroseconds(10);

    uint16_t r1 = analogRead(M1);
    uint16_t r2 = analogRead(M2);

    if(r1 < avg_ls[ch])    rawState &= ~(1UL << ch);
    if(r2 < avg_ls[ch+16]) rawState &= ~(1UL << (ch + 16));
  }

  lineData.state = prevRaw | rawState;
  prevRaw        = rawState;
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
      speed_timer = millis();
      
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
        
    lineVx = 80.0f *cos(finalDegree * DtoR_const);
    lineVy = 80.0f *sin(finalDegree * DtoR_const);   
    return true;
  }
  else{
    first_detect = false;
    lineVx = 0;
    lineVy = 0;
    return false;
  }


}

void readCommand(){
    while(Serial8.available() > 0){
        uint8_t cmd = Serial8.read();
        if(cmd == 0xCC && subState != SUB_ATTACK){
          subState = SUB_LINECAL;
          line_calibrate();
          subState = SUB_IDLE;
        }
        else if(cmd == 0xAA){
          subState = SUB_ATTACK;
        }
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

void setup() {
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
  
 
}
void loop(){
  if(subState != SUB_ATTACK){
        readCommand();  // ✅ 只在非 ATTACK 時讀指令
    }
  else if(subState == SUB_ATTACK){
    readMainPacket();   // 收 vx/vy/omega
    fast_update_line_sensor();
    readBNO085Yaw();
    bool onLine = moveBackInBounds();
    float omg = -omega/4000;
    //if(lineData.state != 0xFFFFFFFF ){MotorStop();}
    static bool useRamp=false;
    if (fabs(gyroData.pitch) > 15) {
    finalVx= 0;finalVy = 0;
    control.robot_heading = 90; 
    Vector_Motion(0, 0,0,1,0); 
    Serial.println("ROBOT PICKED UP - ALL STATES RESET");
    return; 
  }
    if(onLine){
      finalVx = lineVx;
      finalVy = lineVy;
    }
    else{
      finalVx = vx;
      finalVy = vy;
    }     // 白線更新
    if(goal_valid == 0x00){
      Vector_Motion(finalVx, finalVy, 0, 1, 0);
    }
    else{
      Vector_Motion(finalVx, finalVy, omg, 0, 0);
    }
    if(onLine){
      Serial.println("line");
    }
  }
}