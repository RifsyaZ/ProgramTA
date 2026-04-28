#include <Wire.h>

#define SLAVE_ADDR 0x08

float yaw = 1.23;
float Fedback_Angle[4] = {10.1, 20.2, 30.3, 40.4};
float Fedback_Pulse[4] = {100, 200, 300, 400};

uint8_t buffer[36];

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("STM32 Master Ready");
}

void loop() {

  // 🔥 COPY FLOAT KE BUFFER
  memcpy(buffer, Fedback_Angle, 16);
  memcpy(buffer + 16, Fedback_Pulse, 16);
  memcpy(buffer + 32, &yaw, 4);

  // 🔥 DEBUG
  Serial.print("KIRIM STM: ");
  for (int i = 0; i < 36; i++) {
    Serial.print(buffer[i]); Serial.print(" ");
  }
  Serial.println();

  // 🔥 SPLIT KIRIM (18 + 18)
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(buffer, 18);
  Wire.endTransmission();

  delay(5);

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(buffer + 18, 18);
  Wire.endTransmission();

  delay(50);

  // 🔥 TERIMA COMMAND DARI ESP
  Wire.requestFrom(SLAVE_ADDR, 32);

  Serial.print("CMD dari ESP: ");
  while (Wire.available()) {
    Serial.print((char)Wire.read());
  }
  Serial.println();

  delay(100);
}