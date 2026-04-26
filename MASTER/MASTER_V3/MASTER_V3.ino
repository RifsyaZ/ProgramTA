#include "Var.h"
#include "Sensor_Odometry.h"
#include "SwerveDrive.h"
#include "Plan.h"

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);   // Serial Monitor
  Serial1.begin(115200);  // RS485 ke motor
  Serial5.begin(115200);  // komunikasi MCU MPU

  delay(3000);
}

// ==================== LOOP ====================
void loop() {
  MPU1();
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    cmd.toUpperCase();
    if (cmd == "G") {
      TEST(0);
    } else if (cmd == "S") {
      STOP_ALL();
    } else if (cmd == "H") {
      HOME_ALL();
    }
  }
  Serial.println("TES");
}