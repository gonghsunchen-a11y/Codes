#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define SERVICE_UUID               "08001234-1234-4321-4321-123456789012"
#define DATA_CHARACTERISTIC_UUID   "66666666-6666-6666-6666-666666666666"
#define SECRET_KEY                 0x66
#define AUTH_PASSWORD              0x66

typedef struct {
  uint8_t ball_dist;
  uint8_t ball_angle;
  int8_t robot_x;
  int8_t robot_y;
} RobotData;

RobotData slaveData;
bool newData = false;
bool slaveConnected = false;
bool authenticated = false;

BLEServer *pServer = NULL;
BLECharacteristic *pDataCharacteristic = NULL;

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

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    slaveConnected = true;
    authenticated = false;
    Serial.println("裝置連上，等待驗證...");
  }
  void onDisconnect(BLEServer *pServer) {
    slaveConnected = false;
    authenticated = false;
    rgbLEDWrite(255, 0, 0);
    Serial.println("Slave disconnected");
    pServer->getAdvertising()->start();
  }
};

class MyDataCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();

    if (!authenticated) {
      if (value.length() == 4) {
        uint32_t pwd;
        memcpy(&pwd, value.c_str(), 4);
        if (pwd == AUTH_PASSWORD) {
          authenticated = true;
          rgbLEDWrite(0, 255, 0);
          Serial.println("驗證成功");
        } else {
          Serial.println("密碼錯誤，斷線");
          rgbLEDWrite(255, 0, 0);
          pServer->disconnect(0);
        }
      }
      return;
    }

    if (value.length() == 4) {
      slaveData.ball_dist  = value[0] ^ SECRET_KEY;
      slaveData.ball_angle = value[1] ^ SECRET_KEY;
      slaveData.robot_x    = value[2] ^ SECRET_KEY;
      slaveData.robot_y    = value[3] ^ SECRET_KEY;
      newData = true;
    }
  }
};

void setup() {
  Serial.begin(115200);
  rmtInit(38, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000);
  rgbLEDWrite(255, 0, 0);

  BLEDevice::init("Tommy 的 A56");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pDataCharacteristic = pService->createCharacteristic(
    DATA_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pDataCharacteristic->setCallbacks(new MyDataCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("Master ready, waiting for Slave...");
}

void loop() {
  if (newData) {
    newData = false;
    Serial.print("dist:");   Serial.print(slaveData.ball_dist);
    Serial.print(" angle:"); Serial.print(slaveData.ball_angle);
    Serial.print(" x:");     Serial.print(slaveData.robot_x);
    Serial.print(" y:");     Serial.println(slaveData.robot_y);
  }
}
