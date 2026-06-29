#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <math.h>
#include <Robot.h>
#define BALL A16
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
  digitalWrite(LED_BUILTIN,HIGH);
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