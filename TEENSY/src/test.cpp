#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <math.h>

#define pwmPin1 5    // PWM 控制腳
#define pwmPin2 6    // PWM 控制腳
#define DIRA_1 10   // 方向控制腳1
#define DIRB_1 11
#define SLP 12

void setup(){
    pinMode(pwmPin2,OUTPUT);
    pinMode(pwmPin1,OUTPUT);
    pinMode(DIRA_1,OUTPUT);
    pinMode(DIRB_1,OUTPUT);
    pinMode(SLP, OUTPUT);
    digitalWrite(SLP, HIGH);
}

int speed = 0;
int dir = 1;

void loop(){
   uint32_t t = millis() % 1000;

  if (t < 500) {
    speed = 100;
  } else {
    speed = -100;
  }
  int pwmVal = abs(speed) * 255 / 100;

  analogWrite(pwmPin1, pwmVal);
  analogWrite(pwmPin2, pwmVal);

  if (speed > 0) {
    digitalWrite(DIRA_1, HIGH);
    digitalWrite(DIRB_1, LOW);
  }
  else if (speed < 0) {
    digitalWrite(DIRA_1, LOW);
    digitalWrite(DIRB_1, HIGH);
  }
  else {
    digitalWrite(DIRA_1, LOW);
    digitalWrite(DIRB_1, LOW);
  }

  delay(20);
    
} 