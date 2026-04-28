#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>

#define I2C_SLAVE_ADDR 0x08
#define SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "abcd1234-5678-1234-5678-abcdef123456"

volatile char ble_command = '0';
volatile float i2c_yaw = 0;
volatile float i2c_angles[4] = {0, 0, 0, 0};
volatile int i2c_pulses[4] = {0, 0, 0, 0};

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// ================= CALLBACK CONNECT =================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("Client Connected");
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Client Disconnected");
  }
};

// ================= CALLBACK DATA MASUK =================
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0) {
      char cmd = value[0];  // Ambil karakter pertama
      Serial.print("[BLE RX] Command: ");
      Serial.println(cmd);
      
      // Set command untuk dikirim ke Master via I2C
      ble_command = cmd;

      if (cmd == 'F') {
        Serial.println("MAJU");
      }
      else if (cmd == 'B') {
        Serial.println("MUNDUR");
      }
      else if (cmd == 'L') {
        Serial.println("KIRI");
      }
      else if (cmd == 'R') {
        Serial.println("KANAN");
      }
      else if (cmd == 'S') {
        Serial.println("STOP");
      }
    }
  }
};

// ===== I2C SLAVE HANDLERS =====
void onI2CRequest() {
  Wire.write(ble_command);  // Kirim command ke Master
  ble_command = '0';         // Clear
}

void onI2CReceive(int len) {
  if (len == 0) return;
  byte data_type = Wire.read();
  len--;
  
  if (data_type == 0x02) {  // Odometry data
    // Baca yaw
    byte yaw_bytes[4];
    for(int i = 0; i < 4 && Wire.available(); i++) {
      yaw_bytes[i] = Wire.read();
    }
    memcpy((void*)&i2c_yaw, yaw_bytes, 4);
    
    // Baca angles
    for(int j = 0; j < 4; j++) {
      byte angle_bytes[4];
      for(int i = 0; i < 4 && Wire.available(); i++) {
        angle_bytes[i] = Wire.read();
      }
      memcpy((void*)&i2c_angles[j], angle_bytes, 4);
    }
    
    // Baca pulses
    for(int j = 0; j < 4; j++) {
      byte pulse_bytes[2];
      for(int i = 0; i < 2 && Wire.available(); i++) {
        pulse_bytes[i] = Wire.read();
      }
      memcpy((void*)&i2c_pulses[j], pulse_bytes, 2);
    }
    
    Serial.print("[I2C RX] Odometry - YAW: ");
    Serial.print(i2c_yaw);
    Serial.print(" Angles: ");
    for(int i = 0; i < 4; i++) {
      Serial.print(i2c_angles[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  while(Wire.available()) Wire.read();
}

void setup() {
  Serial.begin(115200);
  delay(100);
  
  // ===== INISIALISASI I2C SLAVE =====
  Wire.begin(I2C_SLAVE_ADDR, 10, 8);  // SDA=GPIO10, SCL=GPIO8
  Wire.onRequest(onI2CRequest);
  Wire.onReceive(onI2CReceive);
  Serial.println("[I2C] Slave initialized on 0x08");
  delay(100);

  // ===== INISIALISASI BLE =====
  BLEDevice::init("ESP32-C3-BLE");

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();

  Serial.println("[BLE] Waiting for client...");
}

void loop() {
  if (deviceConnected) {
    String data = "ESP32 READY";
    pCharacteristic->setValue(data.c_str());
    pCharacteristic->notify();
    delay(1000);
  }
}