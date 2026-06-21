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

//Motor1
#define DIR_1 37   // 方向控制腳1
#define pwmPin1 4    // PWM 控制腳

//Motor2
#define DIR_2 11    // 方向控制腳2
#define pwmPin2 6    // PWM 控制腳

//Motor3
#define DIR_3 10    // 方向控制腳3
#define pwmPin3 5    // PWM 控制腳

//Motor4
#define DIR_4 36    // 方向控制腳4
#define pwmPin4 3    // PWM 控制腳

#define SLP1 23    
#define SLP2 12

#define LS_count 32

// --- Data Structures ---

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

struct BallData {
    uint16_t dist = 255; uint16_t angle = 255;
    uint16_t possession = 255; bool valid = false;
    float Vx; float Vy;
};

struct Position {
    int x; // -90 to 90, where 0 is near the center line and 90 is near the goal line
    int y; // -120 to 120
};

struct GoalData {
    uint16_t x = 65535; uint16_t y = 65535;
    uint16_t w = 65535; uint16_t h = 65535;
    bool valid = false;
};

struct USSensor {
    uint16_t dist_b = 0; uint16_t dist_l = 0;
    uint16_t dist_r = 0; uint16_t dist_f = 0;
};

struct RobotStatus {
    float robot_heading = 90.0f;      // Target heading
    //float P_factor = 0.7;         // Pololu Motor
    float P_factor = 0.85f;           // Proportional gain for rotation

    float heading_threshold = 5.0f;  // Deadband (degrees)
    int8_t def_pos = 0;               // Default position state
    bool picked_up = false;           // Lift detection flag
};

// --- Global Variable Declarations (Externs) ---
extern LineData lineData;           
extern MainCoreCommand mainCommand; // Added for MainCoreCommand usage  
extern GyroData gyroData;       
extern RobotStatus robot;        // Added to match Vector_Motion usage
extern uint16_t avg_ls[34];
extern BallData ballData;
extern Position RobotPos;
extern GoalData goalData;
extern USSensor usData;


// --- Core Function Prototypes ---
void sub_core_init();
int  readMux(int ch, int sig);
void update_line_sensor();
void fast_update_line_sensor();
void update_gyro_sensor();
void line_calibrate();
// --- Actuators & IK Prototypes ---
void SetMotorSpeed(uint8_t port, float speed);
void MotorStop();
void RobotIKControl(float vx, float vy, float omega);
void Vector_Motion(float Vx, float Vy, float rot_V);
void FC_Vector_Motion(float WVx, float WVy, float target_heading);
void readMotor();
void readMotorandSendSensors();
void read_cam_and_pos_data();
void readMainPacket();

#endif