#include "sub_core.h"

LineData lineData;           
MainCoreCommand mainCommand; // Added for MainCoreCommand usage  
GyroData gyroData;       
RobotControl control;        // Added to match Vector_Motion usage
uint16_t avg_ls[32]; 
uint16_t max_ls[32];
uint16_t min_ls[32];    

const float linesensorDegreelist[32] = {
    0.00, 11.25, 22.50, 33.75, 45.00, 56.25, 67.50, 78.75, 
    90.00, 101.25, 112.50, 123.75, 135.00, 146.25, 157.50, 168.75, 
    180.00, 191.25, 202.50, 213.75, 225.00, 236.25, 247.50, 258.75, 
    270.00, 281.25, 292.50, 303.75, 315.00, 326.25, 337.50, 348.75
};
//int8_t linesensor_ver_cor[18]={1,2,3,4,5,4,3,2,1,-1,-2,-3,-4,-5,-4,-3,-2,-1};

// --- MATH CONSTANTS & CONTROL PARAMETERS ---
#define DtoR_const 0.0174529f
#define RtoD_const 57.2958f


void sub_core_init() {
Serial8.begin(115200); // For communication with MainCore
    Serial2.begin(115200); // For gyro sensor
    Serial.begin(115200);  // For debugging

    // Multiplexer Control
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    pinMode(M1, INPUT_PULLDOWN);
    pinMode(M2, INPUT_PULLDOWN);

    // Motor Initialization
    // Motor 1
    pinMode(pwmPin1, OUTPUT);
    pinMode(DIRA_1, OUTPUT);
    pinMode(DIRB_1, OUTPUT);
    // Motor 2
    pinMode(pwmPin2, OUTPUT);
    pinMode(DIRA_2, OUTPUT);
    pinMode(DIRB_2, OUTPUT);
    // Motor 3
    pinMode(pwmPin3, OUTPUT);
    pinMode(DIRA_3, OUTPUT);
    pinMode(DIRB_3, OUTPUT);
    // Motor 4
    pinMode(pwmPin4, OUTPUT);
    pinMode(DIRA_4, OUTPUT);
    pinMode(DIRB_4, OUTPUT);

    EEPROM.begin();
    EEPROM.get(0, avg_ls);
}

int readMux(int ch, int sig) {
    digitalWrite(s0, (ch >> 0) & 1);
    digitalWrite(s1, (ch >> 1) & 1);
    digitalWrite(s2, (ch >> 2) & 1);
    digitalWrite(s3, (ch >> 3) & 1);
    delayMicroseconds(10);
    return analogRead(sig);
}

void readBNO085Yaw(){
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];
  gyroData.exist = false; // Reset flag before read attempt

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
      gyroData.exist = true;
    }
    
    if(abs(pitch_raw) <= 18000){
      gyroData.pitch = pitch_raw * 0.01f;
    }
    break; // Process one packet per call
  }
}

void line_calibrate(){
  for(int i=0; i<32; i++){
    max_ls[i] = 0;
    min_ls[i] = 4095;
  }
  while(1){

    if(Serial8.available()){
      if(Serial8.read() == 'E'){
        for(uint8_t i = 0; i < LS_count; i++){
          Serial.print(" min ");Serial.print(i);Serial.print(" = ");Serial.print(min_ls[i]);
          Serial.print(" max ");Serial.print(i);Serial.print(" = ");Serial.print(max_ls[i]);
          Serial.print(" avg ");Serial.print(i);Serial.print(" = ");Serial.print(avg_ls[i]);
          Serial.println("");
        }
        break;
      } 
    }

    for(uint8_t i = 0; i < 32; i++){
    uint16_t reading = readMux(i % 16, (i < 16) ? 1 : 2);

    if(reading > max_ls[i]) max_ls[i] = reading;
    if(reading < min_ls[i]) min_ls[i] = reading;
    } 
  }

  for(uint8_t i = 0; i < LS_count; i++){
      avg_ls[i] = (max_ls[i] + min_ls[i]) / 2;
  }
  EEPROM.put(0, avg_ls);
  Serial8.print('D');
}
void linesensor_update(){
  lineData.state = 0xFFFFFFFF;
  
  for (uint8_t i = 0; i < LS_count; i++) {
    uint16_t reading = readMux(i % 16, (i < 16) ? 1 : 2);;
    
    if (reading < avg_ls[i]) {
      lineData.state &= ~(1UL << i); 
      //Serial.printf("%d,%d",i,reading);
      //Serial.print(" avg ");Serial.print(i);Serial.print(" = ");Serial.print(avg_ls[i]);
      //Serial.println();
    }
  }

  /*for (int i = LS_count - 1; i >= 0; i--) {
    uint8_t bit = (lineData.state >> i) & 1;
    Serial.print(bit);

    if (i % 4 == 0 && i != 0) {
      Serial.print(" "); 
    }
  }
  Serial.println(" ");
  delay(50);*/

}
void moveBackInBounds(){
  //-----LINE SENSOR-----
  float sumX = 0.0f, sumY = 0.0f;
  int count = 0;
  bool linedetected = false;
  for(int i = 0; i < 32; i++){
    if(bitRead(lineData.state, i) == 0){
      //if(i==0){continue;}
      
      //Serial.printf("read%d", i);
      
      float deg = linesensorDegreelist[i];
      sumX += cos(deg * DtoR_const);
      sumY += sin(deg * DtoR_const);
      count++;
      linedetected = true;
    }
  }

  // B : 反彈

  if(linedetected && count > 1){
    float lineDegree = atan2(sumY, sumX) * RtoD_const;
    if (lineDegree < 0){lineDegree += 360;} 
    
    //Serial.print("degree=");Serial.println(lineDegree);

    if (!lineData.first_detect){
      lineData.init_lineDegree = lineDegree;
      lineData.first_detect = true;
      lineData.speed_timer = millis();
      
      Serial.println("LINE DETECTED !!!");
      Serial.print("initlineDegree =");Serial.println(lineData.init_lineDegree);
    }

    lineData.diff = fabs(lineDegree - lineData.init_lineDegree);
    if(lineData.diff > 180){lineData.diff = 360 - lineData.diff;}
    
    //Serial.print("diff =");Serial.println(lineData.diff);


    //-----BACK TO FIELD-----
    float finalDegree;
    if(lineData.diff > lineData.EMERGENCY_THRESHOLD){
      lineData.overhalf = true;
      finalDegree = fmod(lineData.init_lineDegree + 180.0f, 360.0f);
    }
    else{
      lineData.overhalf = false;
      finalDegree = fmod(lineDegree + 180.0f, 360.0f);
    }
    Serial.print("finalDegree =");Serial.println(finalDegree);
        
    lineData.lineVx = 50.0f *cos(finalDegree * DtoR_const);
    lineData.lineVy = 50.0f *sin(finalDegree * DtoR_const);   
  }
  else{
    lineData.first_detect = false;
    lineData.lineVx = 0;
    lineData.lineVy = 0;
  }

  Serial.print("lineVx =");Serial.println(lineData.lineVx);
  Serial.print("lineVy =");Serial.println(lineData.lineVy);
  
}
void SetMotorSpeed(uint8_t port, float speed) {
    // Constrain speed to prevent PWM overflow
    speed = constrain(speed, -255, 255); 
    int pwmVal = abs((int)speed);

    uint8_t p_pwm, p_a, p_b;
    switch(port) {
        case 4: p_pwm = pwmPin1; p_a = DIRA_1; p_b = DIRB_1; break;
        case 3: p_pwm = pwmPin2; p_a = DIRA_2; p_b = DIRB_2; break;
        case 2: p_pwm = pwmPin3; p_a = DIRA_3; p_b = DIRB_3; break;
        case 1: p_pwm = pwmPin4; p_a = DIRA_4; p_b = DIRB_4; break;
        default: return;
    }

    analogWrite(p_pwm, pwmVal);
    digitalWrite(p_a, (speed > 0) ? HIGH : LOW);
    digitalWrite(p_b, (speed < 0) ? HIGH : LOW);
}

void RobotIKControl(float vx, float vy, float omega) {
    // Applying the Inverse Kinematics Matrix
    float p1 = -0.643f * vx + 0.766f * vy + omega;
    float p2 = -0.643f * vx - 0.766f * vy + omega;
    float p3 =  0.707f * vx - 0.707f * vy + omega;
    float p4 =  0.707f * vx + 0.707f * vy + omega;

    SetMotorSpeed(1, p1);
    SetMotorSpeed(2, p2);
    SetMotorSpeed(3, p3);
    SetMotorSpeed(4, p4);
}

void Vector_Motion(float Vx, float Vy, int rot_V) {  
    robot.robot_heading += rot_V; // Update target heading based on input
    float e = robot.robot_heading - (90.0f - gyroData.heading);

    // Normalize error (-180 to 180)
    while (e > 180) e -= 360;
    while (e < -180) e += 360;

    float omega = (fabs(e) > robot.heading_threshold) ? (e * robot.P_factor) : 0;
    RobotIKControl(Vx, Vy, omega);
}

void FC_Vector_Motion(float WVx, float WVy, float target_heading) {
    // 1. Convert to Robot Frame
    float rad = (target_heading - 90.0f) * (M_PI / 180.0f);
    float cos_h = cos(rad);
    float sin_h = sin(rad);

    float robot_vx = WVx * cos_h + WVy * sin_h;
    float robot_vy = -WVx * sin_h + WVy * cos_h;

    // 2. Heading Correction
    float current_gyro_heading = 90.0f - gyroData.heading;
    float e = target_heading - current_gyro_heading;
    
    while (e > 180) e -= 360;
    while (e < -180) e += 360;

    float omega = (fabs(e) > robot.heading_threshold) ? (e * robot.P_factor) : 0;

    RobotIKControl(robot_vx, robot_vy, omega);
}
