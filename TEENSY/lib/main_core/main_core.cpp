#include "main_core.h"

// --- Sensor Data ---
TopMaixData topmaixData; // Maix Position Data (x, y, status, ball info)
BallData ballData;
USSensor usData;
RobotMonitor robotMonitor;
RobotMonitor teammateMonitor;

// Role Switching
int8_t role = 0; // 0: default, 1: defense, 2: offense

struct Point {
    float x;
    float y;
} RobotPos;

// --- OLED OBJECT ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// --- Kicker Constants ---
#define Charge_Pin 33 // Update to your actual pins
#define Kicker_Pin 32

void main_core_init() {
    // Initialize all Hardware Serials
    Serial.begin(115200);
    Serial2.begin(115200); 
    Serial3.begin(115200);
    Serial4.begin(115200); 
    Serial5.begin(921600);
    Serial6.begin(115200); 
    Serial7.begin(115200);
    Serial8.begin(921600);

    Wire.begin();
    Wire.setClock(400000); // Fast I2C for OLED
    // Pin Setups
    pinMode(Charge_Pin, OUTPUT);
    pinMode(Kicker_Pin, OUTPUT);
    
    // Ensure kicker starts safe
    digitalWrite(Charge_Pin, LOW); // Start charging
    digitalWrite(Kicker_Pin, LOW);

    // Button
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_ENTER, INPUT_PULLUP);
    pinMode(BTN_ESC, INPUT_PULLUP);

    pinMode(COM_O1_PIN, INPUT);
    pinMode(COM_O2_PIN, INPUT);
    for (uint8_t i = 0; i < US_COUNT; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
    }
    attachInterrupt(digitalPinToInterrupt(ECHO_F), echoFrontISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ECHO_R), echoRightISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ECHO_B), echoBackISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ECHO_L), echoLeftISR,  CHANGE);
}

void drawMessage(const char* msg) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println(msg);
    display.display();
}

void kicker_control(bool kick) {
    static uint32_t charge_start_time = 0;
    static bool is_charged = false;
    static bool is_charging = false;

    const uint32_t CHARGE_MS = 5000; 
    uint32_t now = millis();

    // 1. Kick Logic (Priority)
    if (kick && is_charged) {
        digitalWrite(Kicker_Pin, HIGH);
        delay(15); // Solenoid pulse
        digitalWrite(Kicker_Pin, LOW);
        
        is_charged = false; 
        is_charging = false;
        Serial.println("KICKED");
        return; // Exit to prevent immediate re-charge trigger in same frame
    }

    // 2. Charging State Machine
    if (!is_charged && !is_charging) {
        // Start a new charge cycle
        charge_start_time = now;
        is_charging = true;
        digitalWrite(Charge_Pin, HIGH);
        Serial.println("CHARGING...");
    }

    if (is_charging) {
        if (now - charge_start_time >= CHARGE_MS) {
            // Charge complete
            digitalWrite(Charge_Pin, LOW);
            is_charging = false;
            is_charged = true;
            Serial.println("READY TO KICK");
        }
    }
}

void get_ball_sensor(){
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

void readTopMaix() {
    uint8_t buffer[12];
    Serial3.write(0xDD);
    while(!Serial3.available());
    Serial3.readBytes(buffer,12);
    topmaixData.valid = 0;
    if(buffer[0] == 0xCC && buffer[11] == 0xEE){
        uint8_t checksum = (buffer[1] + buffer[2] + buffer[3] + buffer[4]+ buffer[5]+ buffer[6]+ buffer[7]+ buffer[8]+ buffer[9]) & 0xFF;
        if(checksum == buffer[10]){
            topmaixData.valid = true;
            topmaixData.x = (int16_t)((uint16_t)buffer[1] | ((uint16_t)buffer[2] << 8));
            topmaixData.y = (int16_t)((uint16_t)buffer[3] | ((uint16_t)buffer[4] << 8));
            topmaixData.status = buffer[5];

            topmaixData.ball_found = buffer[6];
            topmaixData.ball_angle = (uint16_t)buffer[7] | ((uint16_t)buffer[8] << 8);
            topmaixData.ball_dist = buffer[9];
        }
    }
    else{
        topmaixData.valid = false;  // checksum error
    }
}
void roleSwitchControl() {
    if (robotMonitor.role == RobotRole::DEFENSE) {
        // Defense-specific logic
    } else if (robotMonitor.role == RobotRole::OFFENSE) {
        // Offense-specific logic
    }
}

// ==================================================================
// SYSTEM DISPATCHERS (Fire-and-Forget Requests)
// ==================================================================
FASTRUN void triggerBallRequest() {
    Serial6.write(0xDD); // Simply send the token and leave. No blocking wait!
}

FASTRUN void triggerTeammateRequest() {
    Serial6.write(0xBD); // Simply send the token and leave. No blocking wait!
}

// ==================================================================
// THE UNIFIED STREAM PARSER (Zero Collisions, Maximum Speed)
// ==================================================================
FASTRUN void parseSerial6Stream() {
    // Process all bytes currently sitting in the hardware FIFO buffer
    while (Serial6.available() >= 1) {
        uint8_t header = Serial6.peek(); // Inspect the first byte without extraction

        switch (header) {
            
            // CASE 1: Ball Sensor Packet (Expects 5 bytes: 0xDC ... 0xEE)
            case 0xDC: {
                if (Serial6.available() < 5) return; // Wait until full packet arrives in later loops
                uint8_t buf[5];
                Serial6.readBytes(buf, 5);
                
                if (buf[4] == 0xEE) {
                    ballData.angle = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
                    ballData.exist = true;
                }
                break;
            }

            // CASE 2: Teammate Data Packet (Expects 5 bytes: 0xCE ... 0xEE)
            case 0xCE: {
                if (Serial6.available() < 5) return; 
                uint8_t buf[5];
                Serial6.readBytes(buf, 5);
                
                if (buf[4] == 0xEE) {
                    teammateMonitor.pos_x = buf[1];
                    teammateMonitor.pos_y = buf[2];
                    teammateMonitor.ball_angle = ((buf[3] >> 6) & 0x03) * 90; 
                    teammateMonitor.ball_dist = (buf[3] >> 2) & 0x0F; 
                    teammateMonitor.has_possession = (buf[3] >> 1) & 0x01;
                }
                break;
            }

            // CASE 3: ESP32 Async Telemetry Request (Expects 1 byte: 0xAB)
            case 0xAB: {
                Serial6.read(); // Consume the token
                Serial6.write(0xAF); 
                Serial6.write(robotMonitor.pos_x);
                Serial6.write(robotMonitor.pos_y);
                Serial6.write(robotMonitor.has_possession ? 1 : 0);
                Serial6.write(0xEE); 
                break;
            }

            // CASE 4: ESP32 Passive Role Switch Notification (Expects 2 bytes: 0xAF + Role)
            case 0xAF: {
                if (Serial6.available() < 2) return; 
                Serial6.read(); // Consume header
                uint8_t roleByte = Serial6.read();
                
                if (roleByte == 0x0F) robotMonitor.role = RobotRole::OFFENSE;
                else if (roleByte == 0x0D) robotMonitor.role = RobotRole::DEFENSE;
                break;
            }

            // CASE 5: ESP32 Role Switch Handshake Command (Expects 1 byte: 0xBC)
            case 0xBC: {
                Serial6.read(); // Consume token
                Serial6.write(0xAF); 
                Serial6.write(robotMonitor.role == RobotRole::OFFENSE ? 0x0F : 0x0D);
                Serial6.write(0xEE); 
                break;
            }

            // FALLBACK: Garbage handling protection
            default: {
                Serial6.read(); // Stream mismatch, discard 1 byte to realign the parser
                break;
            }
        }
    }
}



void update_all_sensor(){
    get_ball_sensor();
}

void sendMotor(float vx, float vy, float rot_v, int target_heading) {
    uint8_t data[6];
    data[0] = PROTOCAL_HEADER;
    data[1] = (int8_t)vx;
    data[2] = (int8_t)vy;
    // Scale up by 100 so we don't lose decimals (e.g., 0.5 rad/s becomes 50)
    data[3] = (int8_t)(rot_v * 100.0f);
    // Scale down by 10 to fit 0-360 into a single byte (0-36)
    data[4] = (int8_t)(target_heading / 10);
    data[5] = PROTOCAL_END;
    Serial8.write(data, sizeof(data));
}

void stopMotors() {
    sendMotor(0.0f, 0.0f, 0.0f, 90); // Send zero velocities to stop
}


bool UI_Interface(){
    static uint32_t lastDisplayTime = 0;
    switch (robotMonitor.currentState) {
        case RobotState::STATE_READY:
            if (digitalRead(BTN_ENTER) == LOW) {
                Serial8.write(LS_CAL_START); // Command to Sensor Board
                drawMessage("SCANNING");
                delay(500);
                robotMonitor.currentState = RobotState::STATE_CALIBRATING;
                break;
            }

            if (millis() - lastDisplayTime > 100) { // 每 0.1 秒更新一次螢幕
                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(SSD1306_WHITE);
                display.setCursor(0, 0);
                display.printf("ball dist: %d\n", ballData.dist);
                display.printf("ball angle: %d\n",ballData.angle);
                display.printf("us f: %d\n", usData.dist_f);
                display.printf("us l: %d\n", usData.dist_l);
                display.printf("us r: %d\n", usData.dist_r);
                display.printf("us b: %d\n", usData.dist_b);
                display.display();
                lastDisplayTime = millis();
            }
            if(digitalRead(BTN_UP) == LOW){
                display.clearDisplay();
                display.display();
                return false;
            }
            //offense

            //defense

            break;
        case RobotState::STATE_CALIBRATING:
            if (digitalRead(BTN_ESC) == LOW) {
                Serial8.write(LS_CAL_END); // Command to Save
                drawMessage("SAVING...");
                delay(500);
                robotMonitor.currentState = RobotState::STATE_SAVING;
            }
            break;
        case RobotState::STATE_SAVING:
            if(Serial8.available()){
                uint8_t c = Serial8.read();
                if(c == LS_CAL_END){
                    drawMessage("SAVED!");
                    delay(1000); // 讓 SAVED 停一下
                }
                // 回到初始狀態
                drawMessage("READY");
                delay(1000);
                display.clearDisplay();
                robotMonitor.currentState = RobotState::STATE_READY;
            }
            break;
    }
    return true;
}

void triggerUS(uint8_t i) {
  digitalWrite(trigPins[i], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[i], LOW);
}

void updateUS() {
  if (millis() - last_trigger_time >= 50) {
    last_trigger_time = millis();
    current_us = (current_us + 1) % US_COUNT;
    echo_done[current_us] = false;
    triggerUS(current_us);
  }

  for (uint8_t i = 0; i < US_COUNT; i++) {
    if (!echo_done[i]) continue;
    noInterrupts();
    uint32_t dur = echo_duration[i];
    echo_done[i] = false;
    interrupts();
    if (dur > 100 && dur < 12000) {
      float raw = dur * 0.0343f / 2.0f;
      us_invalid_count[i] = 0;
      us_dist_cm[i] = (us_dist_cm[i] < US_INVALID_DISTANCE) ? us_dist_cm[i] * (1.0f - US_FILTER_ALPHA) + raw * US_FILTER_ALPHA: raw;
    } 
    else {
      if (us_invalid_count[i] < US_INVALID_LIMIT) us_invalid_count[i]++;
      if (us_invalid_count[i] >= US_INVALID_LIMIT) us_dist_cm[i] = US_INVALID_DISTANCE;
    }
  }
}