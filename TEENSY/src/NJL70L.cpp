#include <Arduino.h>
#define M1 A0
#define M2 A1

#define s0 A2
#define s1 A3
#define s2 A4
#define s3 A5

int readMux(int ch, int sigPin) {
    digitalWrite(s0, (ch >> 0) & 1);
    digitalWrite(s1, (ch >> 1) & 1);
    digitalWrite(s2, (ch >> 2) & 1);
    digitalWrite(s3, (ch >> 3) & 1);
    if(sigPin == 1) return analogRead(M1);
    if(sigPin == 2) return analogRead(M2);
}

void setup() {
    Serial.begin(115200);
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    pinMode(M1, INPUT_PULLDOWN);
    pinMode(M2, INPUT_PULLDOWN);
}

void loop() {
  //Serial.print("0= ");Serial.println(readMux(0, 1));  // ch=0, M1
  Serial.print("8= ");Serial.println(readMux(8, 1));
    // ch=8, M1
}