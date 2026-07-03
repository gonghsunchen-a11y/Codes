#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <math.h>
#include <Robot.h>
#define BALL A16
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
#define US_FILTER_ALPHA 0.0f
#define US_INVALID_LIMIT 3

enum USIndex { US_FRONT = 0, US_RIGHT = 1, US_BACK = 2, US_LEFT = 3 };
const uint8_t trigPins[US_COUNT] = { TRIG_F, TRIG_R, TRIG_B, TRIG_L };
const uint8_t echoPins[US_COUNT] = { ECHO_F, ECHO_R, ECHO_B, ECHO_L };
volatile uint32_t echo_start[US_COUNT] = {0};
volatile uint32_t echo_duration[US_COUNT] = {0};
volatile bool echo_done[US_COUNT] = {false};

float us_dist_cm[US_COUNT] = {
  US_INVALID_DISTANCE, US_INVALID_DISTANCE, US_INVALID_DISTANCE, US_INVALID_DISTANCE
};
uint8_t us_invalid_count[US_COUNT] = {0};
uint8_t current_us = US_COUNT - 1;
uint32_t last_trigger_time = 0;
uint32_t last_display_time = 0;
//------------------ Ultrasonic ------------------
bool isValidUS(float dist_cm){
  return dist_cm < US_INVALID_DISTANCE;
}

void echoISR(uint8_t i){
  if(digitalRead(echoPins[i]) == HIGH){
    echo_start[i] = micros();
  }
  else{
    echo_duration[i] = micros() - echo_start[i];
    echo_done[i] = true;
  }
}

void echoFrontISR(){ echoISR(US_FRONT); }
void echoRightISR(){ echoISR(US_RIGHT); }
void echoBackISR(){ echoISR(US_BACK); }
void echoLeftISR(){ echoISR(US_LEFT); }

void triggerUS(uint8_t i){
  digitalWrite(trigPins[i], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[i], LOW);
}

void updateFilteredUS(uint8_t i, float raw_dist_cm){
  us_invalid_count[i] = 0;
  if(!isValidUS(us_dist_cm[i])){
    us_dist_cm[i] = raw_dist_cm;
  }
  else{
    us_dist_cm[i] = us_dist_cm[i] * (1.0f - US_FILTER_ALPHA) + raw_dist_cm * US_FILTER_ALPHA;
  }
}

void markInvalidUS(uint8_t i){
  if(us_invalid_count[i] < US_INVALID_LIMIT){
    us_invalid_count[i]++;
  }
  if(us_invalid_count[i] >= US_INVALID_LIMIT){
    us_dist_cm[i] = US_INVALID_DISTANCE;
  }
}

void updateUS(){
  if(millis() - last_trigger_time >= 50){
    last_trigger_time = millis();
    current_us++;
    if(current_us >= US_COUNT){
      current_us = 0;
    }

    echo_done[current_us] = false;
    triggerUS(current_us);
  }

  for(uint8_t i = 0; i < US_COUNT; i++){
    if(echo_done[i]){
      noInterrupts();
      uint32_t duration = echo_duration[i];
      echo_done[i] = false;
      interrupts();

      if(duration > 100 && duration < 15000){
        updateFilteredUS(i, duration * 0.0343f / 2.0f);
      }
      else{
        markInvalidUS(i);
      }
    }
  }
}

void setupUS(){
  for(uint8_t i = 0; i < US_COUNT; i++){
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }

  attachInterrupt(digitalPinToInterrupt(ECHO_F), echoFrontISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_R), echoRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_B), echoBackISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_L), echoLeftISR, CHANGE);
}

void sendMovePacket(
    int16_t vx,
    int16_t vy,
    int8_t aim_offset) {

  vx = constrain(vx, -100, 100);
  vy = constrain(vy, -100, 100);

  uint8_t packet[7];

  packet[0] = 0xAA;
  packet[1] = 0xAA;
  packet[2] = (uint8_t)(int8_t)vx;
  packet[3] = (uint8_t)(int8_t)vy;
  packet[4] = (uint8_t)aim_offset;

  packet[5] =
      packet[2] +
      packet[3] +
      packet[4];

  packet[6] = 0xEE;

  Serial8.write(
      packet,
      sizeof(packet)
  );
}




/*void SetMotorSpeed(uint8_t port, int8_t speed){
  speed = constrain(speed,-1.5 * 50, 1.5 * 50);
  int pwmVal = abs(speed) * 255 / 100;
  switch (port){
    case 1:
      if(speed>0){
        digitalWrite(DIR_1, HIGH);
        analogWrite(pwmPin1, pwmVal);
      }
      else if(speed<0){
        digitalWrite(DIR_1, LOW);
        analogWrite(pwmPin1, pwmVal);
      }
      else{
        analogWrite(pwmPin1, 0);
      }
    case 2:
      if(speed>0){
        digitalWrite(DIR_2, LOW);
        analogWrite(pwmPin2, pwmVal);
      }
      else if(speed<0){
        digitalWrite(DIR_2, HIGH);
        analogWrite(pwmPin2, pwmVal);
      }
      else{
        analogWrite(pwmPin2, 0);
      }
    case 3:
      if(speed>0){
        digitalWrite(DIR_3, LOW);
        analogWrite(pwmPin3, pwmVal);
      }
      else if(speed<0){
        digitalWrite(DIR_3, HIGH);
        analogWrite(pwmPin3, pwmVal);
      }
      else{
        analogWrite(pwmPin3, 0);
      }
    case 4:
      if(speed>0){
        digitalWrite(DIR_4, LOW);
        analogWrite(pwmPin4, pwmVal);
      }
      else if(speed<0){
        digitalWrite(DIR_4, HIGH);
        analogWrite(pwmPin4, pwmVal);
      }
      else{
        analogWrite(pwmPin4, 0);
      }
  }
}
*/

void setup(){
    Robot_Init();
    Serial2.begin(115200);
    pinMode(BALL,INPUT);
     setupUS();
    /*pinMode(DIR_1,OUTPUT);
    pinMode(DIR_2,OUTPUT);
    pinMode(DIR_3,OUTPUT);
    pinMode(DIR_4,OUTPUT);

    pinMode(pwmPin1,OUTPUT);
    pinMode(pwmPin2,OUTPUT);
    pinMode(pwmPin3,OUTPUT);
    pinMode(pwmPin4,OUTPUT);
    
    pinMode(SLP1, OUTPUT);
    pinMode(SLP2, OUTPUT);
    digitalWrite(SLP1, HIGH);
    digitalWrite(SLP2, HIGH);*/
}
/*void MotorStop(){
  analogWrite(pwmPin1, 0);
  analogWrite(pwmPin2, 0);
  analogWrite(pwmPin3, 0);
  analogWrite(pwmPin4, 0);
}*/

void loop() {
  int16_t vx = 0;
  int16_t vy = 80;
  int8_t aim_offset = 0;
  updateUS();
  Serial.print("front US: ");
  Serial.println(us_dist_cm[US_FRONT]);  
  Serial.print("Left US: ");
  Serial.println(us_dist_cm[US_LEFT]);  
  Serial.print("right US: ");
  Serial.println(us_dist_cm[US_RIGHT]);  
  Serial.print("back US: ");
  Serial.println(us_dist_cm[US_BACK]);  

  const float Y_SLOW = 10.0f;
  const float Y_STOP = 50.0f;
  float y = 110 - us_dist_cm[US_FRONT];
  float scale_y = 1.0f;

  if(vy>0 && y>0){
    scale_y = constrain((Y_STOP - fabsf(y)) /
        (Y_STOP - Y_SLOW),
        0.0f,
        1.0f
    );
   scale_y = scale_y * scale_y * scale_y;

    vy = (int16_t)roundf(vy * scale_y);

    if (scale_y < 0.15f) {
      vy = 0;
    }
  }
  vy = (int16_t)roundf(vy * scale_y);
  if (y >= Y_STOP && vy > 0) vy = 0;
  Serial.println(vy);
  sendMovePacket(0, vy, 0);
  /*
  readBNO085Yaw();
  Serial.println(gyroData.heading);*/
  /*

  readMaix();
  
    Serial.print("X=");
    Serial.print(maixPosData.x);
    Serial.print(" Y=");
    Serial.print(maixPosData.y);
    Serial.print(" status=");
    Serial.println(maixPosData.status);
  

  
    Serial.print(" Ball angle=");
    Serial.print(maixPosData.ball_angle);
    Serial.print(" dist=");
    Serial.println(maixPosData.ball_dist);
    */
  //Serial.println(digitalRead(BALL));
  //kicker_control(1);
  //SetMotorSpeed(1, -30);
  //SetMotorSpeed(2, 30);
  //SetMotorSpeed(3, 30);
  //SetMotorSpeed(4, -30);
  /*while(Serial8.available()){
    uint8_t b = Serial8.read();

    if(b < 0x10) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");
  }

  Serial.println();*/
  //FrontCam();if(frontcam.valid){Serial.println("front");}
  //delay(10);
  //Serial.println("test");
  //readMaix();if(maixPosData.valid){Serial.println("omni");}
  //digitalWrite(LED_BUILTIN,HIGH);
  /*
  uint8_t packet[3];
  packet[0] = 0xAA;
  packet[1] = frontcam.offset & 0xFF;
  packet[2] = 0xEE;
  Serial8.write(packet, 3);*/
  /*
  Serial4.write(0xDD);
  delay(100);
  uint32_t start = millis();
  bool got = false;

  while (millis() - start < 200) {
    while (Serial4.available()) {
      got = true;
      uint8_t b = Serial4.read();

      if (b < 11) Serial.print("0");
      Serial.print(b, HEX);
      Serial.print(" ");
    }
  }

  if (!got) {
    Serial.print("no data");
  }

  Serial.println();*/
  //delay(500);
  //FrontCam();if(frontcam.valid){Serial.println(frontcam.x);}
  //kicker_control(1);
  //Serial.print(" y=");Serial.println(maixPosData.y);
  //readBNO085Yaw();
  //Serial.print(gyroData.heading);
  //readBNO085Yaw();
  //Vector_Motion(0,80,0,1,0);
  //Serial.print(" dist=");Serial.println(ballData.dist);
}/*
  Serial3.write(0xDD);

  uint32_t start = millis();
  bool got = false;

  while (millis() - start < 200) {
    while (Serial3.available()) {
      got = true;
      uint8_t b = Serial3.read();

      if (b < 16) Serial.print("0");
      Serial.print(b, HEX);
      Serial.print(" ");
    }
  }

  if (!got) {
    Serial.print("no data");
  }

  Serial.println();
  delay(500);
}*/
  //delay(50);
  //ballsensor();
  //Serial.println(ballData.angle);
  /*
  readMaix();
  
    Serial.print("X=");
    Serial.print(maixPosData.x);
    Serial.print(" Y=");
    Serial.print(maixPosData.y);
    Serial.print(" status=");
    Serial.println(maixPosData.status);
  

  
    Serial.print(" Ball angle=");
    Serial.print(maixPosData.ball_angle);
    Serial.print(" dist=");
    Serial.println(maixPosData.ball_dist);
    */
//}
   /*if(readMaixPosition()){
    Serial.print("x = ");
    Serial.print(maixPosData.x);

    Serial.print(" y = ");
    Serial.print(maixPosData.y);

    Serial.print(" status = ");
    Serial.print(maixPosData.status);

    Serial.print(" valid = ");
    Serial.println(maixPosData.valid);
  }*/
 
  //SetMotorSpeed(4,30);
  
  //SetMotorSpeed(1, 30);
  //SetMotorSpeed(2, -30);
  //SetMotorSpeed(3, -30);
  //SetMotorSpeed(4, 30);
  
  //Vector_Motion(30,0,0,1,0);

/*
void SetMotorSpeed(uint8_t port, int8_t speed){
  speed = constrain(speed,-1.5 * MAX_V, 1.5 * MAX_V);
  int pwmVal = abs(speed) * 255 / 100;
  switch (port){
    case 4:
      //analogWriteFrequency(pwmPin1, 5000); // Set to 5 kHz
      analogWrite(pwmPin1, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_1,HIGH);
        digitalWrite(DIRB_1,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_1,LOW);
        digitalWrite(DIRB_1,HIGH);
      } else{
        digitalWrite(DIRA_1,LOW);
        digitalWrite(DIRB_1,LOW);
      }
      break;
    case 3:
      //analogWriteFrequency(pwmPin2, 5000); // Set to 5 kHz
      analogWrite(pwmPin2, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_2,HIGH);
        digitalWrite(DIRB_2,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_2,LOW);
        digitalWrite(DIRB_2,HIGH);
      } else{
        digitalWrite(DIRA_2,LOW);
        digitalWrite(DIRB_2,LOW);
      }
      break;
    case 2:
      //analogWriteFrequency(pwmPin3, 5000); // Set to 5 kHz
      analogWrite(pwmPin3, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_3,HIGH);
        digitalWrite(DIRB_3,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_3,LOW);
        digitalWrite(DIRB_3,HIGH);
      } else{
        digitalWrite(DIRA_3,LOW);
        digitalWrite(DIRB_3,LOW);
      }
      break;
    case 1:
      //analogWriteFrequency(pwmPin4, 5000); // Set to 5 kHz
      analogWrite(pwmPin4, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_4,HIGH);
        digitalWrite(DIRB_4,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_4,LOW);
        digitalWrite(DIRB_4,HIGH);
      } else{
        digitalWrite(DIRA_4,LOW);
        digitalWrite(DIRB_4,LOW);
      }
      break;
  }
}
*/