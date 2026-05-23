#include <BLEDevice.h>

#define SERVICE_UUID                 "08001234-1234-4321-4321-123456789012"
#define DATA_CHARACTERISTIC_UUID     "66666666-6666-6666-6666-666666666666"
#define SECRET_KEY                   0x66
#define AUTH_PASSWORD                0x66

typedef struct {
  uint8_t ball_dist;
  uint8_t ball_angle;
  int8_t robot_x;
  int8_t robot_y;
} RobotData;

RobotData myData;

static bool doConnect = false;
static bool connected = false;
static BLEAddress *pServerAddress;
static BLERemoteCharacteristic *pDataCharacteristic;

#define TEENSY_SERIAL Serial0

void rgbLEDWrite(uint8_t r, uint8_t g, uint8_t b) {
  rmt_data_t led_data[24];
  int color[3] = {r, g, b};
  int i = 0;
  for (int col = 0; col < 3; col++) {
    for (int bit = 0; bit < 8; bit++) {
      if (color[col] & (1 << (7 - bit))) {
        led_data[i].level0 = 1; led_data[i].duration0 = 8;
        led_data[i].level1 = 0; led_data[i].duration1 = 4;
      } else {
        led_data[i].level0 = 1; led_data[i].duration0 = 4;
        led_data[i].level1 = 0; led_data[i].duration1 = 8;
      }
      i++;
    }
  }
  rmtWrite(38, led_data, RMT_SYMBOLS_OF(led_data), RMT_WAIT_FOR_EVER);
}

class MyClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) {
    connected = true;
    rgbLEDWrite(0, 255, 0);
    Serial.println("BLE Connected");
  }
  void onDisconnect(BLEClient *pclient) {
    connected = false;
    rgbLEDWrite(255, 0, 0);
    Serial.println("BLE Disconnected");
  }
};

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
      Serial.println("Found Master ESP32");
      advertisedDevice.getScan()->stop();
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;
    }
  }
};

bool connectToServer(BLEAddress pAddress) {
  BLEClient *pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallbacks());

  if (!pClient->connect(pAddress)) {
    Serial.println("Connection failed");
    rgbLEDWrite(255, 0, 0);
    return false;
  }

  BLERemoteService *pRemoteService = pClient->getService(SERVICE_UUID);
  if (!pRemoteService) {
    Serial.println("Service not found");
    pClient->disconnect();
    return false;
  }

  pDataCharacteristic = pRemoteService->getCharacteristic(DATA_CHARACTERISTIC_UUID);
  if (!pDataCharacteristic) {
    Serial.println("Characteristic not found");
    pClient->disconnect();
    return false;
  }

  connected = true;
  rgbLEDWrite(0, 255, 0);

  // 連上後先送密碼
  uint32_t pwd = AUTH_PASSWORD;
  pDataCharacteristic->writeValue((uint8_t*)&pwd, 4, false);
  Serial.println("已送出密碼");
  delay(200);

  return true;
}

void setup() {
  Serial.begin(115200);
  TEENSY_SERIAL.begin(115200, SERIAL_8N1, 44, 43);
  delay(2000);

  rmtInit(38, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000);
  rgbLEDWrite(255, 0, 0);

  BLEDevice::init("");

  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30, false);

  Serial.println("Slave ready");
}

void loop() {
  if (doConnect) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("Connected to Master");
    } else {
      Serial.println("Failed, retrying...");
      delay(3000);
      BLEDevice::getScan()->start(5, false);
    }
    doConnect = false;
  }

  if (!connected) {
    rgbLEDWrite(255, 0, 0);
    return;
  }

  TEENSY_SERIAL.write(0xAA);

  unsigned long t = millis();
  while (TEENSY_SERIAL.available() < 6) {
    if (millis() - t > 100) break;
  }

  if (TEENSY_SERIAL.available() >= 6) {
    if (TEENSY_SERIAL.read() == 0xBB) {
      myData.ball_dist  = TEENSY_SERIAL.read();
      myData.ball_angle = TEENSY_SERIAL.read();
      myData.robot_x    = TEENSY_SERIAL.read();
      myData.robot_y    = TEENSY_SERIAL.read();
      uint8_t end       = TEENSY_SERIAL.read();

      if (end == 0xEE) {
        uint8_t buf[4];
        buf[0] = myData.ball_dist  ^ SECRET_KEY;
        buf[1] = myData.ball_angle ^ SECRET_KEY;
        buf[2] = myData.robot_x   ^ SECRET_KEY;
        buf[3] = myData.robot_y   ^ SECRET_KEY;
        pDataCharacteristic->writeValue(buf, 4, false);
        Serial.print("Sent -> dist:"); Serial.print(myData.ball_dist);
        Serial.print(" angle:");       Serial.print(myData.ball_angle);
        Serial.print(" x:");           Serial.print(myData.robot_x);
        Serial.print(" y:");           Serial.println(myData.robot_y);
      }
    }
  }

  delay(50);
}
