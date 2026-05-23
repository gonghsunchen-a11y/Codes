#include <Arduino.h>

#define IR A9

int sampleIR(int pin) {
    int count = 0;
    for(int i = 0; i < 50; i++) {
        if(analogRead(pin) < 10) count++;
        delayMicroseconds(20);
    }
    return count;
}

void setup() {
    pinMode(IR, INPUT);
    Serial.begin(115200);
}

void loop() {
    int strength = sampleIR(IR);
    Serial.println(strength);
    delay(100);
}