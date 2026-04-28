#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SLAVE_ADDR 0x08

// ================= I2C =================
uint8_t rxBuffer[36];
int receivedBytes = 0;

// ================= BLE =================
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
String bleCommand = "";

#define DEVICE_NAME "ESP32-C3-BLE"
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "abcd1234-5678-1234-5678-abcdef123456"

// ================= BLE CALLBACK =================
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { deviceConnected = true; }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    BLEDevice::startAdvertising();
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    bleCommand = String(pCharacteristic->getValue().c_str());
    Serial.print("BLE CMD: ");
    Serial.println(bleCommand);
  }
};

// ================= I2C RECEIVE =================
void onReceive(int len) {
  while (Wire.available()) {
    if (receivedBytes < 36) {
      rxBuffer[receivedBytes++] = Wire.read();
    } else {
      Wire.read();
    }
  }

  // 🔥 kalau data lengkap
  if (receivedBytes >= 36) {

    float A[4], P[4], yaw;

    memcpy(A, rxBuffer, 16);
    memcpy(P, rxBuffer + 16, 16);
    memcpy(&yaw, rxBuffer + 32, 4);

    // 🔥 DEBUG FORMAT SESUAI PERMINTAAN
    Serial.print(A[0]); Serial.print(" ");
    Serial.print(P[0]); Serial.print("        ");

    Serial.print(A[1]); Serial.print(" ");
    Serial.print(P[1]); Serial.print("        ");

    Serial.print(A[2]); Serial.print(" ");
    Serial.print(P[2]); Serial.print("        ");

    Serial.print(A[3]); Serial.print(" ");
    Serial.print(P[3]); Serial.print("        ");

    Serial.println(yaw);

    receivedBytes = 0;
  }
}

// ================= I2C REQUEST =================
void onRequest() {
  String response;

  if (bleCommand != "") {
    response = bleCommand + "\n";
  } else {
    response = "S\n";
  }

  Wire.write(response.c_str());
  bleCommand = "";
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  // ===== BLE =====
  BLEDevice::init(DEVICE_NAME);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_WRITE
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);

  BLEDevice::startAdvertising();

  Serial.println("ESP32 READY (I2C + BLE)");
}

// ================= LOOP =================
void loop() {

  // 🔥 KIRIM DATA KE BLE
  if (deviceConnected && receivedBytes == 0) {
    pCharacteristic->setValue(rxBuffer, 36);
    pCharacteristic->notify();
  }

  delay(50);
} 