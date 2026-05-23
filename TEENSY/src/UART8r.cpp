#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>

void setup(){
  Robot_Init();
}
void loop(){
  if(Serial8.available()){
    String packet = Serial8.readStringUntil('\n');
    packet.trim();

    int firstComma = packet.indexOf(',');
    int secondComma = packet.indexOf(',',firstComma + 1);

    if(firstComma != -1 && secondComma != -1) {
      gyroData.heading = packet.substring(0, firstComma).toInt();
      gyroData.pitch = packet.substring(firstComma + 1, secondComma).toInt();
      gyroData.valid = packet.substring(secondComma + 1).toInt();

      Serial.print("Received heading: "); Serial.println(gyroData.heading);
      Serial.print("Received pitch: "); Serial.println(gyroData.pitch);
      Serial.print("Received valid: "); Serial.println(gyroData.valid);

    } 
  }

}