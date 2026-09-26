#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <math.h>
#include <Robot.h>

#define COM1 36
#define COM2 37
#define eat A16
void setup(){
  Robot_Init();
  pinMode(COM1, INPUT);
  pinMode(COM2, INPUT);
  pinMode(A16, INPUT);
}



void loop(){
  Serial.println(digitalRead(eat));
if(digitalRead(eat) == 0){kicker_control(1);}
else{kicker_control(0);}
} 
