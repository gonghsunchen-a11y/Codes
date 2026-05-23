#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>


void setup(){
  Robot_Init();

  drawMessage("START");
  
}

// 封裝一個刷螢幕的函式，確保不會亂閃



void loop(){
  // --- 1. 處理發送指令 (按鈕) ---
  if (digitalRead(BTN_ENTER) == LOW) {
    Serial8.print('C'); // 傳送校準指令
    drawMessage("SCANNING...");
    delay(200); 
  }
  if (digitalRead(BTN_ESC) == LOW) {
    Serial8.print('E'); // 傳送結束指令
    delay(200); 
  }
  if(Serial8.available()){
    if(Serial8.read() == 'D'){
      Serial.println(Serial8.read());
      drawMessage("SAVED!");
      delay(1000); // 讓 SAVED 停一下
        
        // 回到初始狀態
      drawMessage("READY");
      delay(200);
    }
  }
}


