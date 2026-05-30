#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>

#define TRIG 3
#define ECHO 8

volatile uint32_t echo_start = 0;
volatile uint32_t echo_duration = 0;
volatile bool echo_done = false;

float us_dist_cm = 999;
uint32_t last_trigger_time = 0;

void echoISR(){
  if(digitalRead(ECHO) == HIGH){
    echo_start = micros();
  }
  else{
    echo_duration = micros() - echo_start;
    echo_done = true;
  }
}

void triggerUS(){
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
}
void updateUS(){
  if(millis() - last_trigger_time >= 50){
    last_trigger_time = millis();
    echo_done = false;
    triggerUS();
  }

  if(echo_done){
    noInterrupts();
    uint32_t duration = echo_duration;
    echo_done = false;
    interrupts();

    if(duration > 100 && duration < 25000){
      us_dist_cm = duration * 0.0343f / 2.0f;
    }
    else{
      us_dist_cm = 999;
    }
  }
}

void setup(){
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

   attachInterrupt(digitalPinToInterrupt(ECHO), echoISR, CHANGE);
}


void loop(){
  updateUS();

  Serial.print("dist = ");
  Serial.println(us_dist_cm);
}