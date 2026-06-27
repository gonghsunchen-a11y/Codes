#include <Arduino.h>

#define IR A16

#define SAMPLE_TIME_MS 10
#define IR_THRESHOLD 10

uint32_t start_time = 0;
uint32_t count = 0;
uint32_t sample_total = 0;

void setup() {
  Serial.begin(115200);
  pinMode(IR, INPUT);

  start_time = millis();
}

void loop() {
  int value = analogRead(IR);
  sample_total++;

  if (value < IR_THRESHOLD) {
    count++;
  }

  if (millis() - start_time >= SAMPLE_TIME_MS) {
    Serial.print("count=");
    Serial.print(count);
    Serial.print(" total=");
    Serial.print(sample_total);
    Serial.print(" ratio=");
    Serial.print((float)count / sample_total);
    Serial.print(" last_value=");
    Serial.println(value);

    count = 0;
    sample_total = 0;
    start_time = millis();
  }
}