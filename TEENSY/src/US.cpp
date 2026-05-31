#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>

#define TRIG_F 2
#define ECHO_F 6

#define TRIG_R 3
#define ECHO_R 8

#define TRIG_B 4
#define ECHO_B 9

#define TRIG_L 5
#define ECHO_L 10

#define US_COUNT 4

enum USIndex{
  US_FRONT = 0,
  US_RIGHT = 1,
  US_BACK  = 2,
  US_LEFT  = 3
};

const uint8_t trigPins[US_COUNT] = {
  TRIG_F, TRIG_R, TRIG_B, TRIG_L
};

const uint8_t echoPins[US_COUNT] = {
  ECHO_F, ECHO_R, ECHO_B, ECHO_L
};

volatile uint32_t echo_start[US_COUNT] = {0};
volatile uint32_t echo_duration[US_COUNT] = {0};
volatile bool echo_done[US_COUNT] = {false};

float us_dist_cm[US_COUNT] = {
  999, 999, 999, 999
};

uint8_t current_us = 0;
uint32_t last_trigger_time = 0;

void echoISR(uint8_t i){
  if(digitalRead(echoPins[i]) == HIGH){
    echo_start[i] = micros();
  }
  else{
    echo_duration[i] = micros() - echo_start[i];
    echo_done[i] = true;
  }
}

void echoFrontISR() {
  echoISR(US_FRONT);
}

void echoRightISR() {
  echoISR(US_RIGHT);
}

void echoBackISR() {
  echoISR(US_BACK);
}

void echoLeftISR() {
  echoISR(US_LEFT);
}

void triggerUS(uint8_t i){
  digitalWrite(trigPins[i], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[i], LOW);
}
void updateUS(){
  if(millis() - last_trigger_time >= 50){
    last_trigger_time = millis();

    current_us++;
    if (current_us >= US_COUNT) {
      current_us = 0;
    }

    echo_done[current_us] = false;
    triggerUS(current_us);
  }
  for(uint8_t i= 0; i < US_COUNT; i++){
    if(echo_done[i]){
      noInterrupts();
      uint32_t duration = echo_duration[i];
      echo_done[i] = false;
      interrupts();

      if(duration > 100 && duration < 12000){
        us_dist_cm[i] = duration * 0.0343f / 2.0f;
      }
      else{
        us_dist_cm[i] = 999;
      }
    }
  }
}

void setup(){
  Serial.begin(115200);
  for (uint8_t i = 0; i < US_COUNT; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }

  attachInterrupt(digitalPinToInterrupt(ECHO_F), echoFrontISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_R), echoRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_B), echoBackISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_L), echoLeftISR, CHANGE);
}


void loop(){
  updateUS();

  Serial.print("F=");
  Serial.print(us_dist_cm[US_FRONT]);

  Serial.print(" R=");
  Serial.print(us_dist_cm[US_RIGHT]);

  Serial.print(" B=");
  Serial.print(us_dist_cm[US_BACK]);

  Serial.print(" L=");
  Serial.println(us_dist_cm[US_LEFT]);

  delay(50);
}