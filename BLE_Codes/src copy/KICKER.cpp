#include <Arduino.h>
#include<Robot.h>
//Kicker
#define Charge_Pin 33 //FET1
#define Kicker_Pin 32 //FET2

void setup(){
  Robot_Init();
  pinMode(Kicker_Pin, OUTPUT);
  pinMode(Charge_Pin, OUTPUT);
  digitalWrite(Kicker_Pin, HIGH);
  digitalWrite(Charge_Pin, HIGH);
}
void loop(){}