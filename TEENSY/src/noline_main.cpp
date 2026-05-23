#include <Wire.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

uint8_t goal_valid = 0x00;
float d_last_error = 0;
unsigned long last_time = 0;
float final_omega = 0;

bool hasBall = false;

void setup(){
    Robot_Init();
    drawMessage("START");
}

void loop(){
    readBNO085Yaw();
    readBallCam();
    readussensor();
    readcamera();

    

    // ── 球 vx/vy 計算 ────────────────────────────────
    if(ballData.valid){
        float moving_degree = ballData.angle;
        float offset = 0;
        float ballspeed = constrain(map(ballData.dist, 25, 55, 30, 60), 30, 60);

        if(ballData.dist >= 55){
            moving_degree = ballData.angle;
            offset = 0;
        }
        else{
            //float angleError  = fabs(ballData.angle - 90);
            //float angleFactor = 1.0 - constrain(angleError / 90.0, 0.0, 1.0);
            //ballspeed = ballspeed * (0.4 + 0.6 * angleFactor);
            float side;
            if(ballData.angle >= 84 && ballData.angle <= 98){
                ballspeed = 40;
                offset = 0;
                moving_degree = 90;
            }
            else if(ballData.angle > 98 && ballData.angle < 270){
                side = 1;
                float offsetRatio = exp(-1.5 * (ballData.dist - 50));
                offsetRatio = constrain(offsetRatio, 0.0, 1.0);
                offset = 80 * offsetRatio;
                //offset = 100 * offsetRatio * offsetFactor;
                float angleError = fabs(ballData.angle - 90);
                float smoothWeight = constrain(angleError / 30.0f, 0.0f, 1.0f);
                moving_degree = ballData.angle + (offset * side * smoothWeight);
            }
            else if(ballData.angle < 84 || ballData.angle >= 270){
                side = -1;
                float offsetRatio = exp(-1.5 * (ballData.dist - 50));
                offsetRatio = constrain(offsetRatio, 0.0, 1.0);
                offset = 80 * offsetRatio;
                //offset = 100 * offsetRatio * offsetFactor;
                float angleError = fabs(ballData.angle - 90);
                float smoothWeight = constrain(angleError / 30.0f, 0.0f, 1.0f);
                moving_degree = ballData.angle + (offset * side * smoothWeight);
            }
        }
        if(ballData.dist <= 30 && ballData.angle >= 80 && ballData.angle <= 100){
            moving_degree = 90;
            ballspeed = 80;
            hasBall =true;
        }
        else{
            hasBall = false;
        }
        // ── 球門 omega 計算 ──────────────────────────────
        if(hasBall){
            Serial.println("has ball");
            if(camData.goal_valid){
                goal_valid = 0xFF;
                int error = camData.goal_x - 130;

                unsigned long now = millis();
                float dt = (now - last_time) / 1000.0f;
                float d_error = 0;
                if(dt > 0 && last_time != 0){
                    d_error = (error - d_last_error) / dt;}
                d_last_error = error;
                last_time = now;

                final_omega =-( 0.01f * error + 0.03f * d_error);
                final_omega = constrain(final_omega, -10.0f, 10.0f);
                if(abs(error) < 60) final_omega = 0;
                Serial.println("GOAL--------------------------------------");
                Serial.print("X= ");     Serial.println(camData.goal_x);
                Serial.print("error= "); Serial.println(error);
                Serial.print("d_err= "); Serial.println(d_error);
                Serial.print("omega= "); Serial.println(final_omega);
            }
            else{
                Serial.println("GOAL UNFIND");
                goal_valid  = 0x00;
                final_omega = 0;
                d_last_error  = 0;
                last_time   = 0;
            }
        }
        else{
            goal_valid  = 0x00;
            final_omega = 0;
            d_last_error  = 0;
            last_time   = 0;
        }
        moving_degree = fmod(moving_degree + 360.0f, 360.0f);
        ballData.Vx = (int)round(ballspeed * cos(moving_degree * DtoR_const));
        ballData.Vy = (int)round(ballspeed * sin(moving_degree * DtoR_const));

        float angleError = fabs(ballData.angle - 90);
        float vxWeight = constrain(angleError / 30.0f, 0.0f, 1.0f);
        ballData.Vx = (int)round(ballData.Vx * vxWeight * 0.65);   
        

        //-------------------------------------------------------------------
        //右邊線
        if(usData.dist_r <= 16 ){if(ballData.Vx > 0)ballData.Vx = 0;}
        else if(usData.dist_r <= 20){if(ballData.Vx > 0)ballData.Vx *= 0.5; }
        else if(usData.dist_r <= 24){if(ballData.Vx > 0)ballData.Vx *= 0.7;}
        else{ballData.Vx = ballData.Vx;}
        //左邊線
        if(usData.dist_l <= 26 ){if(ballData.Vx < 0)ballData.Vx = 0;}
        else if(usData.dist_l <= 30){if(ballData.Vx < 0){ballData.Vx *= 0.5;}}
        else if(usData.dist_l <= 34){if(ballData.Vx < 0)ballData.Vx *= 0.7;}
        else{ballData.Vx = ballData.Vx;}
        
        
        //前角落

        if(usData.dist_f <= 25){if(ballData.Vy > 0)ballData.Vy = 0;}
        else if(usData.dist_f <= 27){if(ballData.Vy > 0)ballData.Vy *= 0.6;}
        else if(usData.dist_f <= 29){if(ballData.Vy > 0)ballData.Vy *= 0.8;}
    
        
        if(usData.dist_f <= 25){if(ballData.Vy > 0)ballData.Vy = 0;}
        else if(usData.dist_f <= 27){if(ballData.Vy > 0)ballData.Vy *= 0.6;}
        else if(usData.dist_f <= 29){if(ballData.Vy > 0)ballData.Vy *= 0.8;}
        
        //後角落
        if(usData.dist_l <= 43){
        if(usData.dist_b <= 25){if(ballData.Vy < 0)ballData.Vy = 0;}
        else if(usData.dist_b <= 27){if(ballData.Vy < 0)ballData.Vy *= 0.6;}
        else if(usData.dist_b <= 29){if(ballData.Vy < 0)ballData.Vy *= 0.8;}
        }
        if(usData.dist_r <= 43){
        if(usData.dist_b<= 25){if(ballData.Vy < 0)ballData.Vy = 0;}
        else if(usData.dist_b <= 27){if(ballData.Vy < 0)ballData.Vy *= 0.6;}
        else if(usData.dist_b <= 29){if(ballData.Vy < 0)ballData.Vy *= 0.8;}
        }
        if(usData.dist_b<= 25){if(ballData.Vy < 0)ballData.Vy = 0;}
        if(usData.dist_f<= 25){if(ballData.Vy > 0)ballData.Vy = 0;}
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
    int16_t omega_i = (int16_t)final_omega*100;

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
