#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define SERVICE_UUID               "458063a1-02bf-4664-857e-16c1030be066"
#define DATA_CHARACTERISTIC_UUID   "a5209632-66a9-411d-9353-9be5507790fa"

typedef struct {
  uint8_t ball_dist;
  uint8_t ball_angle;
  int8_t robot_x;
  int8_t robot_y;
} RobotData;

RobotData slaveData;
bool newData = false;
bool slaveConnected = false;

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
    rgbLEDWrite(0, 255, 0);
    Serial.println("Slave connected");
  }
  void onDisconnect(BLEServer *pServer) {
    slaveConnected = false;
    rgbLEDWrite(255, 0, 0);
    Serial.println("Slave disconnected");
    pServer->getAdvertising()->start();
  }
};

class MyDataCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() == sizeof(RobotData)) {
      memcpy(&slaveData, value.c_str(), sizeof(RobotData));
      newData = true;
    }
  }
};

void setup() {
  Serial.begin(115200);
  rmtInit(38, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000);
  rgbLEDWrite(255, 0, 0);

  BLEDevice::init("ESP32_MASTER");
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
    Serial.print("dist:");  Serial.print(slaveData.ball_dist);
    Serial.print(" angle:"); Serial.print(slaveData.ball_angle);
    Serial.print(" x:");     Serial.print(slaveData.robot_x);
    Serial.print(" y:");     Serial.println(slaveData.robot_y);
  }
}
