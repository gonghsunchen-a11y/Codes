#include <BLEDevice.h>

#define SERVICE_UUID                 "458063a1-02bf-4664-857e-16c1030be066"
#define DATA_CHARACTERISTIC_UUID     "a5209632-66a9-411d-9353-9be5507790fa"

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
  return true;
}

void setup() {
  Serial.begin(115200);
  TEENSY_SERIAL.begin(115200, SERIAL_8N1, 44, 43); // RX=44, TX=43
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
  // BLE 連線
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

  // 先發 0xAA 請求資料
  TEENSY_SERIAL.write(0xAA);

  // 等 Teensy 回應
  unsigned long t = millis();
  while (TEENSY_SERIAL.available() < 6) {
    if (millis() - t > 100) break; // 100ms timeout
  }

  // 收 Teensy 資料
  if (TEENSY_SERIAL.available() >= 6) {
    if (TEENSY_SERIAL.read() == 0xBB) {
      myData.ball_dist  = TEENSY_SERIAL.read();
      myData.ball_angle = TEENSY_SERIAL.read();
      myData.robot_x    = TEENSY_SERIAL.read();
      myData.robot_y    = TEENSY_SERIAL.read();
      uint8_t end       = TEENSY_SERIAL.read();

      if (end == 0xEE) {
        pDataCharacteristic->writeValue((uint8_t*)&myData, sizeof(myData), false);
        Serial.print("Sent -> dist:"); Serial.print(myData.ball_dist);
        Serial.print(" angle:");       Serial.print(myData.ball_angle);
        Serial.print(" x:");           Serial.print(myData.robot_x);
        Serial.print(" y:");           Serial.println(myData.robot_y);
      }
    }
  }

  delay(50);
}
