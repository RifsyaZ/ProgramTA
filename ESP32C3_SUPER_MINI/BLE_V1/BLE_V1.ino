#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "abcd1234-5678-1234-5678-abcdef123456"

// ================= CALLBACK CONNECT =================
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Client Connected");
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Client Disconnected");
  }
};

// ================= CALLBACK DATA MASUK =================
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0) {
      Serial.print("Command: ");
      Serial.println(value.c_str());

      if (value == "F") {
        Serial.println("MAJU");
      }
      else if (value == "B") {
        Serial.println("MUNDUR");
      }
      else if (value == "L") {
        Serial.println("KIRI");
      }
      else if (value == "R") {
        Serial.println("KANAN");
      }
      else if (value == "S") {
        Serial.println("STOP");
      }
    }
  }
};

void setup() {
  Serial.begin(115200);

  BLEDevice::init("ESP32-C3-BLE");

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
  pAdvertising->start();

  Serial.println("Waiting for client...");
}

void loop() {
  if (deviceConnected) {
    String data = "ESP32 READY";
    pCharacteristic->setValue(data.c_str());
    pCharacteristic->notify();
    delay(1000);
  }
}