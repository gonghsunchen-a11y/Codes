#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <math.h>
#include <Robot.h>

//Motor1
#define DIR_1 37   // 方向控制腳1
#define pwmPin1 4    // PWM 控制腳

//Motor2
#define DIR_2 10    // 方向控制腳2
#define pwmPin2 5    // PWM 控制腳

//Motor3
#define DIR_3 11    // 方向控制腳3
#define pwmPin3 6    // PWM 控制腳

//Motor4
#define DIR_4 36    // 方向控制腳4
#define pwmPin4 3    // PWM 控制腳


#define SLP1 23    
#define SLP2 12
int speed = 10;
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



void loop(){
   if(readMaixPosition()){
    Serial.print("x = ");
    Serial.print(maixPosData.x);

    Serial.print(" y = ");
    Serial.print(maixPosData.y);

    Serial.print(" status = ");
    Serial.print(maixPosData.status);

    Serial.print(" valid = ");
    Serial.println(maixPosData.valid);
  }


  /*SetMotorSpeed(1, 0);
  SetMotorSpeed(2, 20);
  SetMotorSpeed(3, 0);
  SetMotorSpeed(4, 20);*/
} 

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