#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>


void setup(){
    Robot_Init();
    Serial2.begin(115200);
    drawMessage("TEST READY");
}

void loop(){
    readBNO085Yaw();
    Serial.println(gyroData.pitch);
    delay(300);
}