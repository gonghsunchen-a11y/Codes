#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define possession_Threshold 150

//CAMERA
unsigned long lastCameraUpdate = 0;  // 記錄上次執行時間
const unsigned long interval = 50;  // 20Hz = 每50毫秒一次

//GOAL
//uint32_t lastTargetTime = 0;
//static float rotate = 0;
//static bool isRecovering = false; // 紀錄是否正在處理邊緣回彈

uint8_t goal_valid = 0x00;
float final_omega = 0;
float d_last_error = 0;
unsigned long last_time = 0;

enum State { READY, SCANNING, ATTACK };
State state = READY;

bool btnPressed(int pin){
    static unsigned long last[40] = {0};
    if(digitalRead(pin) == LOW && millis() - last[pin] > 200){
        last[pin] = millis();
        return true;
    }
    return false;
}


void setup(){
  Robot_Init();

}
void loop(){
  if(state == READY || state == SCANNING){
        if(btnPressed(BTN_ENTER)){   // BTN3 → 開始掃線
            state = SCANNING;
            Serial8.write(0xCC);
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(0, 20);
            display.println("SCANNING");
            display.display();
        }
        if(btnPressed(BTN_ESC) && state == SCANNING){  // BTN4 → 存檔
            Serial8.write(0xEE);
            state = READY;
            unsigned long t = millis();
            while(millis() - t < 2000){
                if(Serial8.available() && Serial8.read() == 0xDD) break;
            }
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(0, 20);
            display.println("SAVED\nREADY");
            display.display();
            delay(500);
        }
        
        if(btnPressed(BTN_UP) ){      // BTN1 → 進攻
            state = ATTACK;
            Serial8.write(0xAA);
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(0, 20);
            display.println("ATTACK");
            display.display();
        }
        if(state == READY){
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(0, 20);
            display.println("READY");
            display.display();
        }
        return;  // 非進攻模式不做運算
    }
    readBNO085Yaw();
    readBallCam();
    readussensor();
    readcamera();

    if(camData.goal_valid){
        goal_valid = 0xFF;
        int error = camData.goal_x - 150;

        unsigned long now = millis();
        float dt = (now - last_time) / 1000.0f;
        float d_error = 0;
        if(dt > 0 && last_time != 0){
            d_error = (error - d_last_error) / dt;}
        d_last_error = error;
        last_time = now;

        final_omega = 0.01f * error + 0.05f * d_error;
        final_omega = constrain(final_omega, -10.0f, 10.0f);
        if(abs(error) < 10) final_omega = 0;
        Serial.println("GOAL--------------------------------------");
        Serial.print("X= ");     Serial.println(camData.goal_x);
        Serial.print("H= ");Serial.println(camData.goal_h);
        Serial.print("error= "); Serial.println(error);
        Serial.print("d_err= "); Serial.println(d_error);
        Serial.print("omega= "); Serial.println(final_omega,5);
        
    }
    else{
        Serial.println("GOAL UNFIND");
        goal_valid  = 0x00;
        final_omega = 0;
        d_last_error  = 0;
        last_time   = 0;
    }

    // ── 球 vx/vy 計算 ────────────────────────────────
    if(ballData.valid){
        float moving_degree = ballData.angle;
        float offset = 0;
        float ballspeed = constrain(map(ballData.dist, 30, 60, 30, 60), 30, 60);

        if(ballData.dist >= 60){
            moving_degree = ballData.angle;
            offset = 0;
        }
        else{
            //float angleError  = fabs(ballData.angle - 90);
            //float angleFactor = 1.0 - constrain(angleError / 90.0, 0.0, 1.0);
            //ballspeed = ballspeed * (0.4 + 0.6 * angleFactor);
            float side;
            if(ballData.angle >= 80 && ballData.angle <= 100){
                ballspeed = 50;
                offset = 0;
                moving_degree = 90;
            }
            else if(ballData.angle > 100 && ballData.angle < 270){
                side = 1;
                float offsetRatio = exp(-1.8 * (ballData.dist - 50));
                offsetRatio = constrain(offsetRatio, 0.0, 1.0);
                offset = 80 * offsetRatio;
                //offset = 100 * offsetRatio * offsetFactor;
                float angleError = fabs(ballData.angle - 90);
                float smoothWeight = constrain(angleError / 25.0f, 0.0f, 1.0f);
                moving_degree = ballData.angle + (offset * side);
            }
            else if(ballData.angle < 80 || ballData.angle >= 270){
                side = -1;
                float offsetRatio = exp(-0.8 * (ballData.dist - 55));
                offsetRatio = constrain(offsetRatio, 0.0, 1.0);
                offset = 80 * offsetRatio;
                //offset = 100 * offsetRatio * offsetFactor;
                float angleError = fabs(ballData.angle - 90);
                float smoothWeight = constrain(angleError / 25.0f, 0.0f, 1.0f);
                moving_degree = ballData.angle + (offset * side);
            }
        }
        if(ballData.dist <= 28 && ballData.angle >= 81 && ballData.angle <= 100){
            if(goal_valid){
            moving_degree = 90;
            ballspeed = 60;
            //kicker_control(1);
            if(ballData.dist <= 26){
                kicker_control(1);
            }
            }
            else{
                moving_degree = 90;
                ballspeed = 50;
            }
        }
        
        moving_degree = fmod(moving_degree + 360.0f, 360.0f);
        ballData.Vx = (int)round(ballspeed * cos(moving_degree * DtoR_const));
        ballData.Vy = (int)round(ballspeed * sin(moving_degree * DtoR_const));

        float angleError = fabs(ballData.angle - 90);
        float vxWeight = constrain(angleError / 20.0f, 0.0f, 1.0f);
        ballData.Vx = (int)round(ballData.Vx * vxWeight);
        //if(ballData.angle>= 97 || ballData.angle <=83){}  

        //if(ballData.dist<=36&&ballData.Vx>0 &&ballData.Vx <= 20){ballData.Vx = 20;}
        if(ballData.Vx>0 &&ballData.Vx <= 20){ballData.Vx = 20;}
        //if(ballData.dist<=36&&ballData.Vx<0&&ballData.Vx >= -20){ballData.Vx=-20;}
        if(ballData.Vx<0&&ballData.Vx >= -20){ballData.Vx=-20;}
        //-------------------------------------------------------------------
        //右邊線
        if(usData.dist_r <= 26 ){if(ballData.Vx > 0)ballData.Vx = 0;}
        else if(usData.dist_r <= 30){if(ballData.Vx > 0)ballData.Vx *= 0.5; }
        else if(usData.dist_r <= 34){if(ballData.Vx > 0)ballData.Vx *= 0.7;}
        else{ballData.Vx = ballData.Vx;}
        //左邊線
        if(usData.dist_l <= 26 ){if(ballData.Vx < 0)ballData.Vx = 0;}
        else if(usData.dist_l <= 30){if(ballData.Vx < 0){ballData.Vx *= 0.5;}}
        else if(usData.dist_l <= 34){if(ballData.Vx < 0)ballData.Vx *= 0.7;}
        else{ballData.Vx = ballData.Vx;}
        
        
        
        //後角落
        if(usData.dist_l <= 43){
        if(usData.dist_b <= 25){if(ballData.Vy < 0)ballData.Vy = 0;}
        else if(usData.dist_b <=30){if(ballData.Vy < 0)ballData.Vy *= 0.3;}
        else if(usData.dist_b <= 33){if(ballData.Vy < 0)ballData.Vy *= 0.5;}
        }
        if(usData.dist_r <= 43){
        if(usData.dist_b<= 25){if(ballData.Vy < 0)ballData.Vy = 0;}
        else if(usData.dist_b <= 30){if(ballData.Vy < 0)ballData.Vy *= 0.3;}
        else if(usData.dist_b <= 33){if(ballData.Vy < 0)ballData.Vy *= 0.5;}
        }

        if(usData.dist_b<= 25){if(ballData.Vy < 0)ballData.Vy = 0;}
        if(usData.dist_f<= 25){if(ballData.Vy > 0)ballData.Vy = 0;}
        else if(usData.dist_f <= 35){if(ballData.Vy > 0)ballData.Vy = 0;}
        else if(usData.dist_f <= 50){if(ballData.Vy > 0)ballData.Vy *= 0.5;}
        //-------------------------------------------------------------------
        Serial.println("BALL--------------------------------------");
        Serial.print("angle= ");Serial.println(ballData.angle);
        Serial.print("dist= ");Serial.println(ballData.dist);
        Serial.print("moving= ");Serial.println(moving_degree);
        Serial.print("vx= "); Serial.println(ballData.Vx);
        Serial.print("vy= "); Serial.println(ballData.Vy);
        Serial.print("f= ");Serial.println(usData.dist_f);
        Serial.print("l= ");Serial.println(usData.dist_l);
        Serial.print("b= ");Serial.println(usData.dist_b);
        Serial.print("r= ");Serial.println(usData.dist_r);
    }
    else{
        ballData.Vx = 0;
        ballData.Vy = 0;
         Serial.println("BALL UNFIND");
    }
    
    
    // ── 合併封包送給 Motor sub ───────────────────────
    // AA AA vx_l vx_h vy_l vy_h omega_l omega_h goal_valid checksum EE → 11 bytes
    int16_t vx_i    = (int16_t)ballData.Vx;
    int16_t vy_i    = (int16_t)ballData.Vy;
    int16_t omega_i = (final_omega)* 100;

    uint8_t packet[11];
    packet[0] = 0xAA;
    packet[1] = 0xAA;
    packet[2] = vx_i    & 0xFF;
    packet[3] = (vx_i   >> 8) & 0xFF;
    packet[4] = vy_i    & 0xFF;
    packet[5] = (vy_i   >> 8) & 0xFF;
    packet[6] = omega_i & 0xFF;
    packet[7] = (omega_i >> 8) & 0xFF;
    packet[8] = goal_valid;

    uint8_t sum = 0;
    for(int i = 2; i <= 8; i++) sum += packet[i];
    packet[9]  = sum;
    packet[10] = 0xEE;

    Serial8.write(packet, 11);
}