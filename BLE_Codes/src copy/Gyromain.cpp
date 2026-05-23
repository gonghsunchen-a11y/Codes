#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>

uint8_t goal_valid = 0x00;  // 預設看不到

float last_error = 0;
unsigned long last_time =0;
float final_omega;
float out_omega;
void setup(){
  Robot_Init();

}
float rotate =0;

void loop(){
  readcamera();
  
  if(camData.goal_valid){
      if(camData.goal_x >= 90 && camData.goal_x <= 160){
        rotate=0;
      }
      else if(camData.goal_x < 90){
        rotate = 0.08 * (90-camData.goal_x) / 60.0;
      }
      else if(camData.goal_x > 150){
        rotate = -0.08 * (camData.goal_x-160) / 60.0;
      }
  }
  else{
    rotate =0; 
  }
    Serial.print("X= ");Serial.println(camData.goal_x);
    //Serial.print("Y= ");Serial.println(camData.goal_y);
    //Serial.print("W= ");Serial.println(camData.goal_w);
    //Serial.print("H= ");Serial.println(camData.goal_h);
    Serial.println("------");
    Serial.println(rotate);
  /*
    goal_valid = 0xFF;

  // 2. 判斷邏輯（建議將左右眼邏輯整合，避免互相覆蓋）
  // 假設兩邊都看到，則需要一個綜合判斷
    int error = camData.goal_x - 160;

    unsigned long now = millis();
    float dt = (now - last_time)/1000.f;
    float d_error =0;
    if(dt > 0 && last_time!=0){
      d_error = (error - last_error)/dt;
    }
    last_error = error;
    last_time = now;

    float kP = 0.05f;
    float kD = 0.01f;
    final_omega = constrain(final_omega, -10.0f, 10.0f);
    
    if (abs(error) < 60) {
      final_omega = 0;
    }
    
    Serial.print("X= ");     Serial.println(camData.goal_x);
    Serial.print("error= "); Serial.println(error);
    Serial.print("d_err= "); Serial.println(d_error);
    Serial.print("omega= "); Serial.println(final_omega);
    final_omega = kP*error + kD *d_error;
    */
    /*
  // 比例控制（重點）
    final_omega = error * 0.1;

  // 限制最大輸出
    if (final_omega > 10) final_omega = 10;
    if (final_omega < -10) final_omega = -10;

  // 死區（避免抖）
    if (abs(error) < 70) {
      final_omega = 0;
    }*
    /*
    
    final_omega=0;
    goal_valid = 0x00;
    */

  //Serial.println(final_omega);
  // 3. 封包準備
  uint8_t packet[6];
  int16_t out_omega = (int16_t)(rotate *100);

  packet[0] = 0xAA;
  packet[1] = 0xAA;
  
  // 這裡修正了：把 omega 放入封包，並確保變數名稱正確
  packet[2] = out_omega & 0xFF;
  packet[3] = (out_omega >> 8) & 0xFF;
  // 4. Checksum 計算
  uint8_t sum = 0;
  for(int i = 0; i <= 3; i++){
    sum += packet[i];
  }
  packet[4] = sum;
  packet[5] = 0xEE;

  Serial8.write(packet, 6);
}