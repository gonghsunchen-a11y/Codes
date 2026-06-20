#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <math.h> // Added for sin, cos, and fabs
#include <stdbool.h> // Added for clarity

//Line Sensor
#define EMERGENCY_THRESHOLD 90

//BALL SEARCHING THRESHOLD
#define BALL_Threshold 5
#define TOTAL_BALL_SENSORS 10

//ROBOT MAX SPEED
#define MAX_V 50

//ROBOT DEFENSE PARAMETERS
#define MAX_VX 60
#define MAX_VY 60
#define Def_offset 2.5
#define Back_safe 35//cm
#define Side_safe 45//cm
#define Back_limit 15
#define Side_limit 45

// --- MATH CONSTANTS & CONTROL PARAMETERS ---
#define DtoR_const 0.0174529f
#define RtoD_const 57.2958f

//按鈕
#define BTN_UP 31
#define BTN_DOWN 30
#define BTN_ENTER 27
#define BTN_ESC 26
int _page = 0;      // 0: 主選單, 1: 掃描頁面
int _cursor = 0;    // 選單游標位置
unsigned long _lastPress = 0; 
unsigned long _lastUpdate = 0;
// ------------------ OLED ------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

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
//------------------------------

//US Sensor
#define front_us A15
#define left_us A16
#define back_us A17
#define right_us A14
#define alpha 0.15
float pos_x_f = 0.0;
float pos_y_f = 0.0;


//Kicker
#define Charge_Pin 33 //FET1
#define Kicker_Pin 32 //FET2


// --- GLOBAL OBJECTS & STRUCTS ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

struct GyroData{float heading = 0.0; float pitch = 0.0; bool valid = false;} gyroData;
//struct LineData{uint32_t state = 0x3FFFF; bool valid = false;} lineData;
struct BallData{uint16_t angle = 255; uint16_t possession = 255;uint8_t dist; bool valid = false; float Vx; float Vy;} ballData;
struct MaixPosData {int16_t x = 0;int16_t y = 0;uint8_t status = 0;bool valid = false;bool ball_found = false;uint16_t ball_angle = 0xFFFF;uint8_t ball_dist = 0;} maixPosData;



//float ballDegreelist[18]={0,22.5,45,67.5,87.5,92.5,112.5,135,157.5,180,202.5,225,240,267.5,272.5,300,315,337.5};
float linesensorDegreelist[32] = {
    0.00, 11.25, 22.50, 33.75, 45.00, 56.25, 67.50, 78.75, 
    90.00, 101.25, 112.50, 123.75, 135.00, 146.25, 157.50, 168.75, 
    180.00, 191.25, 202.50, 213.75, 225.00, 236.25, 247.50, 258.75, 
    270.00, 281.25, 292.50, 303.75, 315.00, 326.25, 337.50, 348.75
};
//int8_t linesensor_ver_cor[18]={1,2,3,4,5,4,3,2,1,-1,-2,-3,-4,-5,-4,-3,-2,-1};

// --- ROBOT CONTROL STRUCT (New: For P-control state) ---
struct RobotControl{
    float robot_heading = 90.0;        // Target heading
    float P_factor = 0.7;             // Proportional gain
    float heading_threshold = 5.0;    // Deadband (degrees)
    int8_t vx = 0;
    int8_t vy = 0;
    bool picked_up = false;
} control;


// --- FUNCTION PROTOTYPES ---
// Including prototypes for the new functions and existing ones
void Robot_Init();
void readBNO085Yaw();
void readMaix();
void ballsensor();
void linesensor();
void positionEst();
void showStart();
void showLine();
void showRunScreen();
void showMessage(const char* message, int textSize = 2, int x = -1, int y = -1);
void showSensors(float gyro, int ballAngle);
void SetMotorSpeed(uint8_t port, int8_t speed);
void MotorStop();
void RobotIKControl(int8_t vx, int8_t vy, float omega);
//void Vector_Motion(float Vx, float Vy);
void Vector_Motion(float Vx, float Vy, float rot_V, bool reset,bool useRamp);
void FC_Vector_Motion(int WVx, int WVy, float target_heading);
void Degree_Motion(float moving_degree, int8_t speed);
void kicker_control(bool);
bool menuUpdate() ;
bool white_line_processing();
void readBallCam();

// ******************************************************
// --- FUNCTION IMPLEMENTATIONS (Existing & New) ---
// ******************************************************

void Robot_Init(){
  //pinMode(13, OUTPUT);
  //digitalWrite(13, HIGH);
  
  Serial.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(921600);
  Serial5.begin(921600);
  Serial6.begin(115200);
  Serial7.begin(115200);
  Serial8.begin(115200);
  
  pinMode(DIR_1,OUTPUT);
  pinMode(DIR_2,OUTPUT);
  pinMode(DIR_3,OUTPUT);
  pinMode(DIR_4,OUTPUT);

  pinMode(pwmPin1,OUTPUT);
  pinMode(pwmPin2,OUTPUT);
  pinMode(pwmPin3,OUTPUT);
  pinMode(pwmPin4,OUTPUT);
  
  pinMode(SLP1, OUTPUT);
  pinMode(SLP2, OUTPUT);
  digitalWrite(SLP1, HIGH);
  digitalWrite(SLP2, HIGH);

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_ENTER, INPUT_PULLUP);
  pinMode(BTN_ESC, INPUT_PULLUP);

  pinMode(front_us, INPUT);
  pinMode(back_us, INPUT);
  pinMode(left_us, INPUT);
  pinMode(right_us, INPUT);
  
  pinMode(Kicker_Pin, OUTPUT);
  pinMode(Charge_Pin, OUTPUT);
  digitalWrite(Kicker_Pin, LOW);
  digitalWrite(Charge_Pin, LOW);

  Wire.begin();
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while(1){if(display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){break;}};
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  kicker_control(0);
}

// ── BNO085──────────────
void readBNO085Yaw(){
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];
  gyroData.valid = false; // Reset flag before read attempt

  while (Serial2.available() >= PACKET_SIZE){
    buffer[0] = Serial2.read();
    if(buffer[0] != 0xAA) continue;
    buffer[1] = Serial2.read();
    if(buffer[1] != 0xAA) continue;

    // Read remaining 17 bytes
    for (int i = 2; i < PACKET_SIZE; i++){
      buffer[i] = Serial2.read();
    }

    // --- Checksum: sum of bytes [2..16], mod 256 ---
    uint8_t esti_checksum = 0;
    for (int i = 2; i <= 16; i++){
      esti_checksum += buffer[i];
    }
    esti_checksum %= 256;

    // Compare with buffer[18]
    if(esti_checksum != buffer[18]){
      //Serial.println("Checksum error");
      continue;
    }

    // --- Extract yaw (Little Endian) ---
    int16_t yaw_raw = (int16_t)((buffer[4] << 8) | buffer[3]);
    int16_t pitch_raw = (int16_t)((buffer[6] << 8) | buffer[5]);
    
    //Serial.print("yaw_raw: ");
    //Serial.println(yaw_raw);

    // Convert to degrees if within range
    if(abs(yaw_raw) <= 18000){
      gyroData.heading = yaw_raw * 0.01f;
      gyroData.valid = true;
    }
    
    if(abs(pitch_raw) <= 18000){
      gyroData.pitch = pitch_raw * 0.01f;
    }
    break; // Process one packet per call
  }
}

// ── Cam──────────────
void readMaix() {
    uint32_t start = micros();
  uint8_t buffer[10];
  Serial3.write(0xDD);
  //while(!Serial3.available()){Serial.println("cam");};
  Serial3.readBytes(buffer,10);
  maixPosData.valid = 0;

  if(buffer[0] == 0xCC && buffer[9] == 0xEE){
    uint8_t checksum = (buffer[1] + buffer[2] + buffer[3] + buffer[4]+ buffer[5]+ buffer[6]+ buffer[7]) & 0xFF;
    if(checksum == buffer[8]){
      //Serial.printf("duration: %ld\n", micros() - start);
      maixPosData.valid = true;
      maixPosData.x = (int8_t)(uint8_t)buffer[1];
      maixPosData.y = (int8_t)(uint8_t)buffer[2];
      maixPosData.status = buffer[3];

      maixPosData.ball_found = buffer[4];
      maixPosData.ball_angle = (uint16_t)buffer[5] | ((uint16_t)buffer[6] << 8);
      maixPosData.ball_dist = buffer[7];
    }
  }
  else{
    maixPosData.valid = false;  // checksum error
  }
}
void ballsensor() {
  uint8_t buffer[6];
  Serial6.write(0xDD);
  //while(!Serial6.available()){Serial.println("esp");};
  Serial6.readBytes(buffer,6);
  ballData.valid = 0;
  if(buffer[0] != 0xCC) return;
  if (buffer[5] != 0xEE) return;

  uint8_t found = buffer[1];
  uint16_t angle = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
  uint8_t dist = buffer[4];
  if (!found || angle == 0xFFFF || angle >= 360) {
    return;
  }
  ballData.valid = true;
  ballData.angle = angle;
  ballData.dist = dist;
}

//                  MOTOR
void SetMotorSpeed(uint8_t port, int8_t speed){
  speed = constrain(speed,-1.5 * 50, 1.5 * 50);
  int pwmVal = abs(speed) * 255 / 100;
  switch (port){
    case 1:
      //Serial.print("1 ");Serial.println(speed);
      if(speed<0){
        digitalWrite(DIR_1, LOW);
        analogWrite(pwmPin1, pwmVal);
      }
      else if(speed>0){
        digitalWrite(DIR_1, HIGH);
        analogWrite(pwmPin1, pwmVal);
      }
      else{
        analogWrite(pwmPin1, 0);
      }
      //Serial.print("1 ");Serial.println(digitalRead(DIR_1));
    break;
    case 2:
      if(speed<0){
        digitalWrite(DIR_2, LOW);
        analogWrite(pwmPin2, pwmVal);
      }
      else if(speed>0){
        digitalWrite(DIR_2, HIGH);
        analogWrite(pwmPin2, pwmVal);
      }
      else{
        analogWrite(pwmPin2, 0);
      }
      //Serial.print("2 ");Serial.println(pwmVal);
      break;
    case 3:
      if(speed<0){
        digitalWrite(DIR_3, LOW);
        analogWrite(pwmPin3, pwmVal);
      }
      else if(speed>0){
        digitalWrite(DIR_3, HIGH);
        analogWrite(pwmPin3, pwmVal);
      }
      else{
        analogWrite(pwmPin3, 0);
      }
      //Serial.print("3 ");Serial.println(pwmVal);
      break;
    case 4:
      if(speed<0){
        digitalWrite(DIR_4, LOW);
        analogWrite(pwmPin4, pwmVal);
      }
      else if(speed>0){
        digitalWrite(DIR_4, HIGH);
        analogWrite(pwmPin4, pwmVal);
      }
      else{
        analogWrite(pwmPin4, 0);

      }
      //Serial.print("4 ");Serial.println(pwmVal);
    break;
  }
}

void MotorStop(){
  analogWrite(pwmPin1, 0);
  analogWrite(pwmPin2, 0);
  analogWrite(pwmPin3, 0);
  analogWrite(pwmPin4, 0);
}
/*
void RobotIKControl(float vx, float vy, float omega){
  // Note: Cast omega to int8_t for consistent data types in the IK control matrix
  float p1 = -0.643f * vx + 0.766f * vy + omega;
    float p2 = -0.643f * vx - 0.766f * vy + omega;
    float p3 =  0.707f * vx - 0.707f * vy + omega;
    float p4 =  0.707f * vx + 0.707f * vy + omega;
  p1 *= 0.7;
  p4 *= 0.9;
  //p4 *= 0.85;
  //Serial.print("p1= ");Serial.println(p1);
  //Serial.print("p2= ");Serial.println(p2);
  //Serial.print("p3= ");Serial.println(p3);
  //Serial.print("p4= ");Serial.println(p4);
  SetMotorSpeed(1, p1);
  SetMotorSpeed(2, p2);
  SetMotorSpeed(3, p3);
  SetMotorSpeed(4, p4);
}
*/
// ── 馬達校正參數──────────────
struct MotorCal {
    float scale;
    int8_t dead;
};

constexpr MotorCal CAL[5] = {
    {},
    {1.00f, 0},  // M1
    {1.00f,  0},  // M2
    {1.00f,  0},  // M3
    {1.00f, 0},  // M4
};

static int8_t applyMotorCal(float raw, const MotorCal& cal) {
    if (raw == 0) return 0;
    float scaled = raw * cal.scale;
    float out = scaled + (scaled > 0 ? cal.dead : -cal.dead);
    return (int8_t)constrain(out, -127, 127);
}
static float current_p[5] = {0,0,0,0,0};

void RobotIKControl(float vx, float vy, float omega,bool useRamp){
    float target[5];
    
    target[1] = applyMotorCal(-0.643f * vx + 0.766f * vy + omega, CAL[1]);
    target[2] = applyMotorCal(-0.643f * vx - 0.766f * vy + omega, CAL[2]);
    target[3] = applyMotorCal( 0.707f * vx - 0.707f * vy + omega, CAL[3]);
    target[4] = applyMotorCal( 0.707f * vx + 0.707f * vy + omega, CAL[4]);
    
    float ramp = 5.0f;

    for(int i = 1; i <= 4; i++){
      if(useRamp){
        float diff = target[i] - current_p[i];
        if(fabs(diff) <= ramp)
            current_p[i] = target[i];
        else
            current_p[i] += (diff > 0) ? ramp : -ramp;
      }
      else{
        current_p[i] = target[i];
      }
      SetMotorSpeed(i, (int8_t)current_p[i]);
    }
}
/*
void RobotIKControl(float vx, float vy, float omega){
    float p1 = -0.643f * vx + 0.766f * vy + omega;
    float p2 = -0.643f * vx - 0.766f * vy + omega;
    float p3 =  0.707f * vx - 0.707f * vy + omega;
    float p4 =  0.707f * vx + 0.707f * vy + omega;

    SetMotorSpeed(1, applyMotorCal(p1, CAL[1]));
    SetMotorSpeed(2, applyMotorCal(p2, CAL[2]));
    SetMotorSpeed(3, applyMotorCal(p3, CAL[3]));
    SetMotorSpeed(4, applyMotorCal(p4, CAL[4]));
}

*/
void Vector_Motion(float Vx, float Vy, float rot_V, bool reset,bool useRamp) {
  float omega = 0.0;
  if(reset && rot_V == 0){
    control.robot_heading = 90;
    float current_gyro_heading = gyroData.heading;
    float sensor_heading = 90.0 - current_gyro_heading;
    float e = control.robot_heading - sensor_heading;
    if(fabs(e) > control.heading_threshold){
     omega = e * control.P_factor;
    }
  }
  else{
    control.robot_heading += rot_V; // Update target heading based on input
    if(control.robot_heading > 135){
      control.robot_heading =  135;  
    }
    else if(control.robot_heading < 45){
      control.robot_heading = 45;  
    }
    //.printf("control.robot_heading%f\n",control.robot_heading);
    
    float e = control.robot_heading - (90.0f - gyroData.heading);

    // Normalize error (-180 to 180)
    while (e > 180) e -= 360;
    while (e < -180) e += 360;
    omega = (fabs(e) > control.heading_threshold) ? (e * control.P_factor) : 0;
    omega *= 0.5;
  }
  //Serial.printf("Vx%f, Vy%f", Vx, Vy);
  if(useRamp){
    RobotIKControl(Vx, Vy, omega,1);
  }
  else{
    RobotIKControl(Vx, Vy, omega,0);
  }
}
/*
void Vector_Motion(float Vx, float Vy){  
  float omega = 0.0;
  float current_gyro_heading = gyroData.heading;
  float sensor_heading = 90.0 - current_gyro_heading;
  float e = control.robot_heading - sensor_heading;
  if(fabs(e) > control.heading_threshold){
    omega = e * control.P_factor;
  }
  RobotIKControl(Vx, Vy, omega);
}
*/
/*void Vector_Motion(float Vx, float Vy, float target_offset){  
  float omega = 0.0;
  float current_gyro_heading = gyroData.heading;
  float sensor_heading = 90.0 - current_gyro_heading;
  float final_target = 90 + target_offset;
  float e = final_target - sensor_heading;
  if (e > 180) e -= 360;
  if (e < -180) e += 360;
  if(fabs(e) > control.heading_threshold){
      omega = e * control.P_factor;
  }
  if(control.robot_heading > 135){
    control.robot_heading =  135;  
  } 
  else if(control.robot_heading < 45){
    control.robot_heading = 45;  
  }
  RobotIKControl(Vx, Vy, omega);

}
*/
void FC_Vector_Motion(int WVx, int WVy, float target_heading) {
    // 1. Convert gyro to Radians (math functions use radians)
    float rad = (target_heading-90)* (M_PI / 180.0);
    float cos_h = cos(rad);
    float sin_h = sin(rad);

    // 2. Rotate World Vectors to Robot Frame
    int8_t robot_vx = (int8_t)(WVx * cos_h + WVy * sin_h);
    int8_t robot_vy = (int8_t)(-WVx * sin_h + WVy * cos_h);
    //Serial.printf("robot %d, %d\n", robot_vx, robot_vy);
    // 3. Calculate Heading Correction (Omega)
    float omega = 0;
    float current_gyro_heading = 90 - gyroData.heading;
    // Normalize error to find the shortest path to target_heading
    float e = target_heading - current_gyro_heading;
    while (e > 180) e -= 360;
    while (e < -180) e += 360;

    if (fabs(e) > control.heading_threshold) {
        omega = e * control.P_factor ;
    }
    //Serial.printf("omege%d\n", omega);
    // 4. Send to IK Control
    RobotIKControl(robot_vx, robot_vy, (int8_t)omega);
}

void Degree_Motion(float moving_degree, int8_t speed){
  if(moving_degree > 360.0 || moving_degree < 0.0){
      MotorStop();
  }
  float moving_degree_rad = moving_degree * DtoR_const;
  float Vx = cos(moving_degree_rad) * speed;
  float Vy = sin(moving_degree_rad) * speed;
  Vector_Motion(Vx, Vy, 0,1,false);
}


void kicker_control(bool kick = false){
  static uint64_t charge_start = 0;
  static uint64_t last_charge_done = 0;
  static bool charging_state = false;

  const uint32_t CHARGE_DURATION = 5000;   // ms needed to charge
  const uint32_t CHARGE_TIMEOUT  = 8000;  // ms before recharging automatically

  uint64_t now = millis();

  // Auto-recharge if too long since last charge
  if(charging_state && (now - last_charge_done > CHARGE_TIMEOUT)){
    charging_state = false;
  }

  // Start charging if not charged and not already charging
  if(!charging_state && charge_start == 0){
    charge_start = now;
    //Serial.println("Charge");
    digitalWrite(Charge_Pin, HIGH);
    digitalWrite(Kicker_Pin, LOW);
  }

  // Stop charging when duration is met
  if(charge_start != 0 && (now - charge_start >= CHARGE_DURATION)){
    digitalWrite(Charge_Pin, LOW);
    digitalWrite(Kicker_Pin, LOW);
    //Serial.println("Charge End");
    charging_state = true;
    charge_start = 0;
    last_charge_done = now;
  }

  // Perform kick if charged
  if(kick && charging_state){
    digitalWrite(Kicker_Pin, HIGH);
    delay(10);
    digitalWrite(Kicker_Pin, LOW);
    //delay(10);
    // After kick, reset to recharge again
    Serial.println("kick");
    charging_state = false;
  }
}

/*
void readBallCam(){
    
    static uint16_t buffer[6] = {0};
    static uint16_t idx = 0;
    while(Serial4.available()){
        uint16_t b = Serial4.read();
        if(idx == 0 && b != 0xCC){continue;} //wait for 0xCC
        buffer[idx++] = b;

        if(idx == 6){ //裝包 共6組
            if(buffer[0] == 0xCC && buffer[5] == 0xEE){
              ballData.angle = (uint16_t)buffer[1] | ((uint16_t)buffer[2] << 8);
              ballData.dist  = (uint16_t)buffer[3] | ((uint16_t)buffer[4] << 8);
            
               if(ballData.angle != 65535 && ballData.dist != 65535)
                ballData.valid = true;
               else{
                ballData.valid = false;
               }  //無球
            }
            else{
                ballData.valid = false;
            }  //無數據
            idx = 0;  // reset buffer
        }  
    }
}
void linesensor(){
  uint8_t buffer[7];
  Serial7.write(0xdd);
  while(!Serial7.available());
  Serial7.readBytes(buffer,7);
  lineData.valid = false;
  if(buffer[0] != 0xaa) return;
  if(buffer[0] == 0xAA && buffer[6] == 0xEE){
    uint8_t checksum = (buffer[1] + buffer[2] + buffer[3] + buffer[4]) & 0xFF;
    if(checksum == buffer[5]){
      lineData.valid = true;
      lineData.state = buffer[1] | (buffer[2] << 8) | (buffer[3] << 16) | (buffer[4] << 24);     
      if(lineData.state != 0b111111111111111111){
        Vector_Motion(0,0);  // Stop robot if line detected
      }
    }
  }
  else{
    lineData.valid = false;  // checksum error
  }
}

void readussensor(){
  // static variables remember their values between calls
  static float dist_b_f = 0.0f;
  static float dist_l_f = 0.0f;
  static float dist_r_f = 0.0f;
  static float dist_f_f = 0.0f;

  // read raw ADC and convert to cm (or mm depending on your scaling)
  float dist_b_raw = analogRead(back_us) * 520.0f / 1024.0f;
  float dist_l_raw = analogRead(left_us) * 520.0f / 1024.0f;
  float dist_r_raw = analogRead(right_us) * 520.0f / 1024.0f;
  float dist_f_raw = analogRead(front_us) * 520.0f / 1024.0f;
  // complementary (low-pass) filtering
  dist_b_f = alpha * dist_b_f + (1.0f - alpha) * dist_b_raw;
  dist_l_f = alpha * dist_l_f + (1.0f - alpha) * dist_l_raw;
  dist_r_f = alpha * dist_r_f + (1.0f - alpha) * dist_r_raw;
  dist_f_f = alpha * dist_f_f + (1.0f - alpha) * dist_f_raw;
  // assign filtered values to struct
  usData.dist_b = dist_b_f;
  usData.dist_l = dist_l_f;
  usData.dist_r = dist_r_f;
  usData.dist_f = dist_f_f;
}
*/