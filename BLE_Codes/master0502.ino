#include <BLEDevice.h>

// 要连接的服务器的服务和特征 UUID （必须与服务器代码一致）
#define SERVICE_UUID "458063a1-02bf-4664-857e-16c1030be066"
#define BRIGHTNESS_CHARACTERISTIC_UUID "a5209632-66a9-411d-9353-9be5507790fa"

// 全局变量
static boolean doConnect = false;
static boolean connected = false;
static BLEAddress *pServerAddress;
static BLERemoteCharacteristic *pRemoteCharacteristic;

// 电位器相关定义
const int potentiometerPin = 7;  // 电位器连接到 GPIO 7
uint8_t lastBrightness = 0;       // 存储上一次发送的亮度值 (0-255)

class MyClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) {}

  void onDisconnect(BLEClient *pclient) {
    connected = false;
    Serial.println("onDisconnect: Client Disconnected");
  }
};

// 扫描回调类，当发现 BLE 设备时被调用
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // 找到设备，检查它是否包含正在寻找的服务。
    if (advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
      Serial.print("Found target server by Service UUID: ");
      Serial.println(advertisedDevice.getAddress().toString().c_str());

      // 停止扫描
      advertisedDevice.getScan()->stop();

      // 保存服务器地址，并设置连接标志
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;
    }
  }
};

// 连接服务器的函数
bool connectToServer(BLEAddress pAddress) {
  Serial.print("Connecting to ");
  Serial.println(pAddress.toString().c_str());

  // 创建 BLE 客户端
  BLEClient *pClient = BLEDevice::createClient();
  Serial.println(" - Client created");

  pClient->setClientCallbacks(new MyClientCallbacks());

  // 连接到远程 BLE 服务器
  if (!pClient->connect(pAddress)) {
    Serial.println(" - Connection failed");
    return false;
  }
  Serial.println(" - Connected to server");

  // 获取服务器上的服务
  BLERemoteService *pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find service UUID: ");
    Serial.println(SERVICE_UUID);
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Service found");

  // 获取服务中的特征
  pRemoteCharacteristic = pRemoteService->getCharacteristic(BRIGHTNESS_CHARACTERISTIC_UUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find characteristic UUID: ");
    Serial.println(BRIGHTNESS_CHARACTERISTIC_UUID);
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Characteristic found");

  connected = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE LED Brightness Controller (Client)...");

  // 初始化 BLE， 作为客户端时， 设备名称不是必需的，因为它只扫描而不广播自己。
  BLEDevice::init("");

  // 获取扫描对象并设置回调
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);  // 主动扫描
  pBLEScan->start(30, false);     // 开始扫描，持续 30 秒
}

void loop() {
  // 如果我们收到了连接指令并且尚未连接，则尝试连接
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("Successfully connected to the server!");
      doConnect = false;  // 清除连接指令
    } else {
      Serial.println("Failed to connect to the server. Rescanning after 3 seconds...");
      delay(3000);
      BLEDevice::getScan()->start(5, false);  // 重新开始扫描 5 秒
    }
  }

  // 如果已连接，则读取电位器并发送数据
  if (connected) {
    uint8_t data[5] = {0xAA, 0xCC, 0xEE, 0x01, 0x02};
    Serial.println("send");
    pRemoteCharacteristic->writeValue((uint8_t*)data, sizeof(data), false);
    delay(100);  // 每 100 毫秒检查一次
  } else {
    // 如果断开连接，则重新扫描
    if (!doConnect) {
      Serial.println("Disconnected. Rescanning...");
      BLEDevice::getScan()->start(5, false);
    }
  }
}
