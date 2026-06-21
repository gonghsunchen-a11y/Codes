#include "sub_core.h"

// 1. Define the actual memory for these variables here
LineData lineData;           
MainCoreCommand mainCommand; // Added for MainCoreCommand usage  
GyroData gyroData;       
RobotStatus control;        // Added to match Vector_Motion usage
MainCoreCommand mainCommand;
GoalData goalData;
USSensor usData;

// --- Sensor Data ---
BallData ballData;
Position RobotPos;

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
    Serial8.begin(921600); // For communication with MainCore
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

void update_line_sensor(){
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
}

void fast_update_line_sensor(){
  static uint32_t prevRaw = 0xFFFFFFFF;
  uint32_t rawState       = 0xFFFFFFFF;

  for(uint8_t ch = 0; ch < 16; ch++){
    digitalWriteFast(s0, (ch >> 0) & 1);
    digitalWriteFast(s1, (ch >> 1) & 1);
    digitalWriteFast(s2, (ch >> 2) & 1);
    digitalWriteFast(s3, (ch >> 3) & 1);
    /*might be allowed to removed*/
    //delayMicroseconds(10);

    uint16_t r1 = analogRead(M1);
    uint16_t r2 = analogRead(M2);

    if(r1 < avg_ls[ch])    rawState &= ~(1UL << ch);
    if(r2 < avg_ls[ch+16]) rawState &= ~(1UL << (ch + 16));
  }

  lineData.state = prevRaw | rawState;
  prevRaw        = rawState;
}


void update_gyro_sensor(){
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
/* --- Actuators Part --- */

void SetMotorSpeed(uint8_t port, float speed){
  speed = constrain(speed,-1.5 * 50, 1.5 * 50);
  int pwmVal = abs(speed) * 255 / 100;
  switch (port){
    case 1:
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
    break;
  }
}

void MotorStop(){
  analogWrite(pwmPin1, 0);
  analogWrite(pwmPin2, 0);
  analogWrite(pwmPin3, 0);
  analogWrite(pwmPin4, 0);
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

void Vector_Motion(float Vx, float Vy, float rot_V) {  
    robot.robot_heading += rot_V; // Update target heading based on input
    if(robot.robot_heading > 135){
      robot.robot_heading =  135;  
    }
    else if(robot.robot_heading < 45){
      robot.robot_heading = 45;  
    }
    Serial.printf("robot.robot_heading%f\n",robot.robot_heading);
    
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

void readMotor() {
    // Process all available bytes to find a valid packet
    while (Serial8.available() >= 6) {
        if (Serial8.peek() != PROTOCAL_HEADER) {
            Serial8.read(); 
            continue;
        }

        uint8_t buf[6];
        Serial8.readBytes(buf, 6);

        if (buf[5] == PROTOCAL_END) {
            // 1. Update motor values
            mainCommand.vx = (float)((int8_t)buf[1]);
            mainCommand.vy = (float)((int8_t)buf[2]);
            mainCommand.rot_v = (float)((int8_t)buf[3]) / 100.0f;
            mainCommand.heading = (uint16_t)((int8_t)buf[4]) * 10;
            break; 
        }
    }
}

void readMotorandSendSensors() {
    // Process all available bytes to find a valid packet
    while (Serial8.available() >= 6) {
        if (Serial8.peek() != PROTOCAL_HEADER) {
            Serial8.read(); 
            continue;
        }

        uint8_t buf[6];
        Serial8.readBytes(buf, 6);

        if (buf[5] == PROTOCAL_END) {
            // 1. Update motor values
            mainCommand.vx = (float)((int8_t)buf[1]);
            mainCommand.vy = (float)((int8_t)buf[2]);
            mainCommand.rot_v = (float)((int8_t)buf[3]) / 100.0f;
            mainCommand.heading = (uint16_t)((int8_t)buf[4]) * 10;

            // 2. Prepare FRESH sensor data to send back immediately
            uint8_t res[7];
            res[0] = PROTOCAL_HEADER;
            float temp = fmod((90 - gyroData.heading + 360), 360);
            res[1] = (uint8_t)(temp / 10.0);
            res[2] = (uint8_t)(lineData.state & 0xFF);
            res[3] = (uint8_t)((lineData.state >> 8) & 0xFF);
            res[4] = (uint8_t)((lineData.state >> 16) & 0xFF);
            res[5] = (uint8_t)((lineData.state >> 24) & 0xFF);
            res[6] = PROTOCAL_END;

            // 3. Respond exactly once per received command
            Serial8.write(res, 7);
            break; 
        }
    }
}

void readMainPacket() {
    static uint8_t buffer[20];
    static int index = 0;

    while (Serial8.available() > 0) {
        uint8_t b = Serial8.read();
        buffer[index++] = b;
        if (index == 20) {
            index = 0;
            if (buffer[19] != 0xEE) continue;
            uint8_t sum = 0;
            for (int i = 2; i <= 17; i++) sum += buffer[i];
            if (sum != buffer[18]) continue;

            ballData.angle = (int16_t)((buffer[3] << 8) | buffer[2]);
            ballData.dist  = (int16_t)((buffer[5] << 8) | buffer[4]);
            ballData.valid = buffer[6] == 0xFF;
            goalData.x     = (int16_t)((buffer[8] << 8) | buffer[7]);
            goalData.valid = buffer[9] == 0xFF;
            // usData.dist_ｆ       = (int16_t)((buffer[11] << 8) | buffer[10]);
            // usData.dist_ｌ       = (int16_t)((buffer[15] << 8) | buffer[14])
            // usData.dist_ｒ       = (int16_t)((buffer[16] << 8) | buffer[17]);
            usData.dist_b     = (int16_t)((buffer[13] << 8) | buffer[12]);
        }
    }
}