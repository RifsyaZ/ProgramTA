// ==================== I2C COMMUNICATION LIBRARY ====================
// I2C Master: STM32 (MASTER_V3)
// I2C Slave:  ESP32-C3 (BLE_V1)
// Address: 0x08, Speed: 400kHz
// SCL: STM32 PB6 ↔ ESP32 GPIO8
// SDA: STM32 PB7 ↔ ESP32 GPIO10

#include <Wire.h>

#define I2C_SLAVE_ADDR 0x08
#define I2C_SPEED 400000
#define I2C_SDA_PIN 7
#define I2C_SCL_PIN 6

// ==================== I2C MASTER FUNCTIONS ====================
void I2C_Master_Init() {
  Wire.begin();
  Wire.setClock(I2C_SPEED);
  Serial.println("[I2C] Master initialized");
}

// Baca command BLE dari ESP32
char I2C_ReadBLECommand() {
  Serial.println("[I2C] Requesting BLE command from ESP32...");
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(0x01);  // Request command
  int err = Wire.endTransmission();
  if (err != 0) {
    Serial.print("[I2C] Request error: ");
    Serial.println(err);
  }
  
  delayMicroseconds(100);
  
  Wire.requestFrom(I2C_SLAVE_ADDR, 1);
  if (Wire.available()) {
    char cmd = Wire.read();
    Serial.print("[I2C] Received raw command: ");
    Serial.println(cmd);
    return cmd;
  }
  return '0';
}

// Kirim odometry data ke ESP32
void I2C_SendOdometry(float yaw_val, 
                      float ang_fl, float ang_fr, float ang_rl, float ang_rr,
                      int pls_fl, int pls_fr, int pls_rl, int pls_rr) {
  Serial.println("[I2C] Sending odometry data to ESP32...");
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(0x02);  // Data type: odometry
  
  // Kirim yaw (4 bytes)
  byte* ptr = (byte*)&yaw_val;
  for(int i = 0; i < 4; i++) Wire.write(ptr[i]);
  
  // Kirim angles (4 × 4 bytes)
  float angles[4] = {ang_fl, ang_fr, ang_rl, ang_rr};
  for(int j = 0; j < 4; j++) {
    ptr = (byte*)&angles[j];
    for(int i = 0; i < 4; i++) Wire.write(ptr[i]);
  }
  
  // Kirim pulses (4 × 2 bytes)
  int pulses[4] = {pls_fl, pls_fr, pls_rl, pls_rr};
  for(int j = 0; j < 4; j++) {
    ptr = (byte*)&pulses[j];
    for(int i = 0; i < 2; i++) Wire.write(ptr[i]);
  }
  
  int status = Wire.endTransmission();
  if (status != 0) {
    Serial.print("[I2C] Send odometry error: ");
    Serial.println(status);
  } else {
    Serial.println("[I2C] Odometry data sent");
  }
}

// ==================== I2C SLAVE VARIABLES ====================
volatile char ble_command = '0';
volatile float i2c_yaw = 0;
volatile float i2c_angles[4] = {0, 0, 0, 0};
volatile int i2c_pulses[4] = {0, 0, 0, 0};

// ==================== I2C SLAVE HANDLERS ====================
void onI2CRequest() {
  Wire.write(ble_command);
  ble_command = '0';
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
  }
  
  // Clear buffer
  while(Wire.available()) Wire.read();
}

// ==================== I2C SLAVE INIT ====================
void I2C_Slave_Init() {
  Wire.begin(I2C_SLAVE_ADDR, 10, 8);  // SDA=GPIO10, SCL=GPIO8
  Wire.onRequest(onI2CRequest);
  Wire.onReceive(onI2CReceive);
  Serial.println("[I2C] Slave initialized on 0x08");
}

void I2C_SetBLECommand(char cmd) {
  ble_command = cmd;
}

float I2C_GetYaw() {
  return i2c_yaw;
}

void I2C_GetAngles(float* angles) {
  memcpy(angles, (void*)i2c_angles, 4 * sizeof(float));
}

void I2C_GetPulses(int* pulses) {
  memcpy(pulses, (void*)i2c_pulses, 4 * sizeof(int));
}
