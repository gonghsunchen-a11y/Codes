#include <Servo.h>
#include <Arduino.h>
#include <Robot.h>
Servo ESC;  // Create ESC control object

void setup() {
    Serial.begin(9600);
    Robot_Init();
    drawMessage("Start");
    // Attach ESC to pin 9
    ESC.attach(23,1000,2000);

    // Start at minimum throttle
    Serial.println("Initializing ESC...");
    ESC.writeMicroseconds(1000);
    delay(2000);  // Give the ESC time to detect the signal
    Serial.println("Started at 1000 µs");

    // Slowly ramp from 1000 to 1500 for safe arming
    for (int signal = 1000; signal <= 1500; signal += 10) {
        ESC.writeMicroseconds(signal);
        Serial.println(signal);
        delay(20);  // Slow enough for ESC to recognize change
    }
    ESC.writeMicroseconds(1500);
    Serial.println("Initialization complete.");
    delay(5000);  // Optional pause before starting loop
}

void loop() {
    /*
    for (int signal = 1450; signal <= 1500; signal += 10) {
        ESC.writeMicroseconds(signal);
        Serial.println(signal);
        delay(2000);  // Slow enough for ESC to recognize change
        //Stop at 1460
    }
    for (int signal = 1550; signal >= 1500; signal -= 10) {
        ESC.writeMicroseconds(signal);
        Serial.println(signal);
        delay(2000);  // Slow enough for ESC to recognize change
        //Stop at 1500
    }*/
    ESC.writeMicroseconds(1550);
}