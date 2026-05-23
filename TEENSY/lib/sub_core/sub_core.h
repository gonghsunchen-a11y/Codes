#ifndef SUB_CORE_H
#define SUB_CORE_H

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <Dual_Core_Config.h>

//MUTIPLEXER PINS
#define M1 A0
#define M2 A1

#define s0 A2
#define s1 A3
#define s2 A4
#define s3 A5

// Motor 1 Pins
#define pwmPin4 23  // PWM 控制腳
#define DIRA_4 37    // 方向控制腳1
#define DIRB_4 36

// Motor 2 Pins
#define pwmPin3 5    // PWM 控制腳
#define DIRA_3 9   // 方向控制腳1
#define DIRB_3 6

// Motor 3 Pins
#define pwmPin2 10    // PWM 控制腳
#define DIRA_2 12   // 方向控制腳1
#define DIRB_2 11

// Motor 4 Pins
#define pwmPin1 2    // PWM 控制腳
#define DIRA_1 4   // 方向控制腳1
#define DIRB_1 3

struct MainCoreCommand
{
  float vx = 0.0f;
  float vy = 0.0f;
  float deg = 0.0f;
  enum command_type{ACTUAE, CALIBRATE}type;
};

struct LineData{
  float lineVx = 0, lineVy = 0;
  bool first_detect = false;
  float init_lineDegree = -1;
  float diff = 0;
  bool overhalf = false;
  int EMERGENCY_THRESHOLD = 90;
  uint32_t speed_timer = 0;    
  uint32_t line_state = 0xFFFFFFFF;
};

struct GyroData{
  float heading = 0.0f;
  float pitch = 0.0f;
  bool exist = false;
};

struct RobotControl{
    float robot_heading = 90.0;        // Target heading
    float P_factor = 0.5;             // Proportional gain
    float heading_threshold = 10.0;     // Deadband (degrees)
    int8_t vx = 0;
    int8_t vy = 0;
    bool picked_up = false;
};

// --- Global Variable Declarations (Externs) ---
extern LineData lineData;           
extern MainCoreCommand mainCommand; // Added for MainCoreCommand usage  
extern GyroData gyroData;       
extern RobotControl control;        // Added to match Vector_Motion usage
extern uint16_t avg_ls[32]; 
extern uint16_t max_ls[32];
extern uint16_t min_ls[32];



// --- Actuators & IK Prototypes ---
void sub_core_init();
void line_calibrate();
void linesensor_update();
void moveBackInBounds();
void readBNO085Yaw();
void SetMotorSpeed(uint8_t port, float speed);
void RobotIKControl(float vx, float vy, float omega);
void Vector_Motion(float Vx, float Vy, int rot_V);
void FC_Vector_Motion(float WVx, float WVy, float target_heading);
uint32_t readfrom_MainCore();

#endif