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

// 服务器回调类，用于处理连接和断开事件
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("Client connected successfully");
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Client disconnected, restarting advertisement");
    // 立即重新开始广播，以便客户端可以重新连接
    pServer->getAdvertising()->start();
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
}

void loop() {
  if (newDataAvailable) {
    // 重置标志位，防止重复处理
    newDataAvailable = false;

    Serial.print("Ball dist received: ");
    Serial.println(received.ball_dist);
    Serial.println(received.ball_angle);
    Serial.println(received.robot_x);
    Serial.println(received.robot_y);
    Serial.println(received.mode);
  }
}
