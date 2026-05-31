#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>

void setup(){
  Robot_Init();
  Serial2.begin(115200);
}
void loop(){
  readBNO085Yaw();
  
  String packet = String(gyroData.heading) + "," + String(gyroData.pitch) + "," + String(gyroData.valid);
  Serial8.println(packet);
}