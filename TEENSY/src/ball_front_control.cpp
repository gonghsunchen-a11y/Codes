#include "Arduino.h"
#include <Robot.h>

#define trig1 32
#define trig2 31
#define trig3 30
#define trig4 29
#define trig5 28
#define trig6 27
#define trig7 26

// Debounce confirmation frames
#define CONFIRM_FRAMES 3

void setup() {
    Robot_Init();
    pinMode(trig1, INPUT_PULLDOWN);
    pinMode(trig2, INPUT_PULLDOWN);
    pinMode(trig3, INPUT_PULLDOWN);
    pinMode(trig4, INPUT_PULLDOWN);
    pinMode(trig5, INPUT_PULLDOWN);
    pinMode(trig6, INPUT_PULLDOWN);
    pinMode(trig7, INPUT_PULLDOWN);
}

void loop() {
    readBNO085Yaw();

    // Read all pins once — consistent state this cycle
    bool t[7];
    t[0] = digitalReadFast(trig1);
    t[1] = digitalReadFast(trig2);
    t[2] = digitalReadFast(trig3);
    t[3] = digitalReadFast(trig4);
    t[4] = digitalReadFast(trig5);
    t[5] = digitalReadFast(trig6);
    t[6] = digitalReadFast(trig7);

    int activeCount = t[0]+t[1]+t[2]+t[3]+t[4]+t[5]+t[6];

    // Shoot confirmation counter — prevent false triggers
    static int shootCounter = 0;
    static int noballCounter = 0;

    float vx = 0, vy = 0;

    if (activeCount == 7) {
        shootCounter++;
        noballCounter = 0;
        if (shootCounter >= CONFIRM_FRAMES) {
            // Confirmed: possessed + goal aligned → shoot
            vy = MAX_V;
            vx = 0;
        }
    } else {
        shootCounter = 0;

        // Weighted position → smooth steering
        // trig1=far left, trig4=center, trig7=far right
        float weights[7] = {-1.0, -0.5, -0.2, 0, 0.2, 0.5, 1.0};
        float weightedPos = 0;
        for (int i = 0; i < 7; i++) {
            if (t[i]) weightedPos += weights[i];
        }

        if (activeCount == 0) {
            // No ball — search behavior
            noballCounter++;
            vx = 0; vy = 0; // or add search spin
        } else {
            noballCounter = 0;
            vx = weightedPos * MAX_V;
            vy = MAX_V * 0.4; // always move forward while chasing
        }
    }

    Serial.printf("GPIO: %d%d%d%d%d%d%d | Vx: %.2f Vy: %.2f\n",
        t[1],t[2],t[3],t[4],t[5],t[6],t[7], vx, vy);

    Vector_Motion(vx, vy, 0,1,1);
}