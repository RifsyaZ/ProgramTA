#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "Var.h"
#include "MPU.h"

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  Wire.begin();
  Wire.setClock(400000);
  setMPU();

  Serial.println("MPU6050 Slave Ready");
}

void loop() {
  // Cek perintah dari Serial Monitor
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "CAL") {
      runCalibration();
    }
  }
  
  // Cek perintah CAL dari Master
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "CAL") {
      runCalibration();
    }
  }

  // Update MPU dan kirim YAW
  MPU1();
}