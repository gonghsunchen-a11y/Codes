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
#define US_INVALID_DISTANCE 999.0f
#define US_FILTER_ALPHA 0.35f
#define US_INVALID_LIMIT 3

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
  US_INVALID_DISTANCE, US_INVALID_DISTANCE, US_INVALID_DISTANCE, US_INVALID_DISTANCE
};

float coord_x_cm = US_INVALID_DISTANCE;
float coord_y_cm = US_INVALID_DISTANCE;
uint8_t current_us = US_COUNT - 1;
uint32_t last_trigger_time = 0;
uint32_t last_display_time = 0;
uint8_t us_invalid_count[US_COUNT] = {0};

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

void updateFilteredUS(uint8_t i, float raw_dist_cm){
  us_invalid_count[i] = 0;

  if(us_dist_cm[i] >= US_INVALID_DISTANCE){
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

bool isValidUS(float dist_cm){
  return dist_cm < US_INVALID_DISTANCE;
}

void updateUSCoordinate(){
  if(isValidUS(us_dist_cm[US_LEFT]) && isValidUS(us_dist_cm[US_RIGHT])){
    coord_x_cm = (us_dist_cm[US_LEFT] - us_dist_cm[US_RIGHT]) / 2.0f;
  }
  else{
    coord_x_cm = US_INVALID_DISTANCE;
  }

  if(isValidUS(us_dist_cm[US_BACK]) && isValidUS(us_dist_cm[US_FRONT])){
    coord_y_cm = (us_dist_cm[US_BACK] - us_dist_cm[US_FRONT]) / 2.0f;
  }
  else{
    coord_y_cm = US_INVALID_DISTANCE;
  }
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
        float raw_dist_cm = duration * 0.0343f / 2.0f;
        updateFilteredUS(i, raw_dist_cm);
      }
      else{
        markInvalidUS(i);
      }

      updateUSCoordinate();
    }
  }
}

void printUSValue(float dist_cm){
  if(!isValidUS(dist_cm)){
    display.print(" ---.-");
  }
  else{
    if(dist_cm < 100.0f){
      display.print(" ");
    }
    if(dist_cm < 10.0f){
      display.print(" ");
    }
    display.print(dist_cm, 1);
  }
}

void printCoordValue(float coord_cm){
  if(!isValidUS(coord_cm)){
    display.print("---");
  }
  else{
    display.print((int)round(coord_cm));
  }
}

void drawUSLine(const char* label, uint8_t index, uint8_t y){
  display.setCursor(0, y);
  display.print(label);
  printUSValue(us_dist_cm[index]);
  display.print(" cm");
}

void showUSDistances(){
  if(millis() - last_display_time < 100){
    return;
  }
  last_display_time = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 22);
  display.print("X:");
  printCoordValue(coord_x_cm);
  display.print(" Y:");
  printCoordValue(coord_y_cm);

  drawUSLine("Front:", US_FRONT, 30);
  drawUSLine("Right:", US_RIGHT, 38);
  drawUSLine("Back: ", US_BACK, 46);
  drawUSLine("Left: ", US_LEFT, 54);

  display.display();
}

void setup(){
  Serial.begin(115200);
  Wire.begin();
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while(1);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();

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
  showUSDistances();
}
