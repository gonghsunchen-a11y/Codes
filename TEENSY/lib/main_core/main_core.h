#ifndef MAINCORE_H
#define MAINCORE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Dual_Core_Config.h"

// --- OLED Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// --- Button Pins ---
#define BTN_UP 31
#define BTN_DOWN 30
#define BTN_ENTER 27
#define BTN_ESC 26

// --- Ultrasonic ---
// Ultrasonic pins
#define TRIG_F 2  #define ECHO_F 6
#define TRIG_R 3  #define ECHO_R 8
#define TRIG_B 4  #define ECHO_B 9
#define TRIG_L 5  #define ECHO_L 10

#define US_COUNT            4
#define US_INVALID_DISTANCE 999.0f
#define US_FILTER_ALPHA     0.35f
#define US_INVALID_LIMIT    3

#define COM_O1_PIN 37
#define COM_O2_PIN 36

#define RIGHT_SLOW_X 65
#define RIGHT_STOP_X 80
#define LEFT_SLOW_X -65
#define LEFT_STOP_X -80
#define SIDE_SLOW_EXP 2.0f

// --- 1. Blueprints (Struct Definitions) ---
struct TopMaixData {int16_t x = 0;int16_t y = 0;uint8_t status = 0;bool valid = false;bool ball_found = false;uint16_t ball_angle = 0xFFFF;uint8_t ball_dist = 0;};

struct BallData {
    uint16_t dist = 65535; uint16_t angle = 65535;
    bool exist = false;
};

struct USSensor {
    uint16_t dist_b = 0; uint16_t dist_l = 0;
    uint16_t dist_r = 0; uint16_t dist_f = 0;
};

enum class RobotRole : int8_t {
    DEFAULT = 0,
    DEFENSE = 1,
    OFFENSE = 2
};

enum class RobotState : uint8_t { STATE_READY, STATE_CALIBRATING, STATE_SAVING };

// 2. Robot state structure
struct RobotMonitor {
    RobotRole role = RobotRole::DEFAULT; // Uses the custom enum type with a default value
    RobotState currentState = RobotState::STATE_READY;
    int8_t pos_x = 0; 
    int8_t pos_y = 0;
    bool has_possession = false;
    float vx = 0.0f; 
    float vy = 0.0f; 
    float rot_v = 0.0f;
    uint16_t heading = 0; // Target heading in degrees
    //for teammate data:
    uint8_t ball_dist = 0; // 0-15 (4 bits)
    uint8_t ball_angle = 0; // 0-255 (8 bits)
};


enum USIndex   { US_FRONT = 0, US_RIGHT = 1, US_BACK = 2, US_LEFT = 3 };

MainState state = READY;

const uint8_t trigPins[US_COUNT] = { TRIG_F, TRIG_R, TRIG_B, TRIG_L };
const uint8_t echoPins[US_COUNT] = { ECHO_F, ECHO_R, ECHO_B, ECHO_L };

volatile uint32_t echo_start[US_COUNT]    = {0};
volatile uint32_t echo_duration[US_COUNT] = {0};
volatile bool     echo_done[US_COUNT]     = {false};

float   us_dist_cm[US_COUNT]       = { US_INVALID_DISTANCE, US_INVALID_DISTANCE, US_INVALID_DISTANCE, US_INVALID_DISTANCE };


// --- 3. External Variables ---
// These tell the compiler "The actual memory for these is in main.cpp"
extern TopMaixData topmaixData;
extern BallData ballData;
extern USSensor usData;
extern RobotMonitor robotMonitor;
extern RobotMonitor teammateMonitor;
extern Adafruit_SSD1306 display;


// --- Function Prototypes ---
void main_core_init();
void drawMessage(const char* msg);
void kicker_control(bool kick);
void readTopMaix();
void roleSwitchControl();
void get_ball_sensor();
void parseSerial6Stream();
void listenToEspCommands();
void sendRoleSwitchCommand();
void getRoleSwitchCommand();
void update_all_sensor();
void sendMotor(float vx, float vy, float rot_v, int heading);
void stopMotors();
bool UI_Interface();
void triggerUS(uint8_t i);
void updateUS();


void echoISR(uint8_t i) {
  if (digitalRead(echoPins[i]) == HIGH) echo_start[i] = micros();
  else { echo_duration[i] = micros() - echo_start[i]; echo_done[i] = true; }
}
void echoFrontISR() { echoISR(US_FRONT); }
void echoRightISR() { echoISR(US_RIGHT); }
void echoBackISR()  { echoISR(US_BACK);  }
void echoLeftISR()  { echoISR(US_LEFT);  }

#endif