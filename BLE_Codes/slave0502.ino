#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// 全局变量
BLEServer *pServer = NULL;
BLECharacteristic *pBrightnessCharacteristic = NULL;

bool deviceConnected = false;
bool newDataAvailable = false;
typedef struct {
  uint8_t ball_dist;
  int8_t ball_angle;
  uint8_t robot_x;
  uint8_t robot_y;
  uint8_t mode;
} Packet;

Packet received;

// LED 相关定义
const int ledPin = 7; // LED 连接到 GPIO 7

// 为服务和特征定义唯一的 UUID
#define SERVICE_UUID "458063a1-02bf-4664-857e-16c1030be066"
#define BRIGHTNESS_CHARACTERISTIC_UUID "a5209632-66a9-411d-9353-9be5507790fa"

void rgbLEDWrite(uint8_t red_val, uint8_t green_val, uint8_t blue_val) {
  rmt_data_t led_data[24];
  // default WS2812B color order is G, R, B
  int color[3] = {red_val, green_val, blue_val};
  int i = 0;
  for (int col = 0; col < 3; col++) {
    for (int bit = 0; bit < 8; bit++) {
      if ((color[col] & (1 << (7 - bit)))) {
        // HIGH bit
        led_data[i].level0 = 1;     // T1H
        led_data[i].duration0 = 8;  // 0.8us
        led_data[i].level1 = 0;     // T1L
        led_data[i].duration1 = 4;  // 0.4us
      } else {
        // LOW bit
        led_data[i].level0 = 1;     // T0H
        led_data[i].duration0 = 4;  // 0.4us
        led_data[i].level1 = 0;     // T0L
        led_data[i].duration1 = 8;  // 0.8us
      }
      i++;
    }
  }
  rmtWrite(38, led_data, RMT_SYMBOLS_OF(led_data), RMT_WAIT_FOR_EVER);
}

// 服务器回调类，用于处理连接和断开事件
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("Client connected successfully");
    rgbLEDWrite(0,255,0);
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Client disconnected, restarting advertisement");
    // 立即重新开始广播，以便客户端可以重新连接
    pServer->getAdvertising()->start();
    rgbLEDWrite(255,0,0);
  }
};

// 特征回调类，用于处理客户端的写入请求
class MyBrightnessCallbacks : public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic *pCharacteristic) {

    String value = pCharacteristic->getValue();

    if (value.length() == sizeof(Packet)) {

      memcpy(&received, value.c_str(), sizeof(Packet));

      newDataAvailable = true;
    }
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32 BLE LED Controller");

  // 设置引脚为输出
  pinMode(ledPin, OUTPUT);

  // 1. 初始化 BLE 设备
  BLEDevice::init("ESP32_LED");

  // 2. 创建 BLE 服务器并设置回调
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // 3. 创建 BLE 服务
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // 4. 创建 BLE 特征
  pBrightnessCharacteristic = pService->createCharacteristic(
    BRIGHTNESS_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE // 只允许写入
  );

  // 为特征设置写入回调
  pBrightnessCharacteristic->setCallbacks(new MyBrightnessCallbacks());

  // 5. 启动服务
  pService->start();
  Serial.println("BLE service started");

  // 6. 启动广播
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pServer->getAdvertising()->start();

  Serial.println("Advertisement started, ready for connections"); 
  rmtInit(38, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
}

void loop() {
  if (newDataAvailable) {
    // 重置标志位，防止重复处理
    newDataAvailable = false;
    Serial.println(received.ball_dist);
    Serial.println(received.ball_angle);
    Serial.println(received.robot_x);
    Serial.println(received.robot_y);
    Serial.println(received.mode);
  }
}
