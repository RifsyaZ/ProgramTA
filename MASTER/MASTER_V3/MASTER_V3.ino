#include "Var.h"
#include "Sensor_Odometry.h"
#include "SwerveDrive.h"
#include "Plan.h"

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);   // Serial Monitor
  Serial1.begin(115200);  // RS485 ke motor
  Serial2.begin(115200);  // Feedback dari ID1
  Serial3.begin(115200);  // Feedback dari ID2
  Serial4.begin(115200);  // Feedback dari ID3
  Serial5.begin(115200);  // komunikasi MCU MPU
  Serial6.begin(115200);  // komunikasi KE ID 4

  delay(3000);
}

// ==================== LOOP ====================
void loop() {
  fedback();
}
// if (Serial.available()) {
//   String cmd = Serial.readStringUntil('\n');
//   cmd.trim();
//   if (cmd.length() == 0) return;

//   cmd.toUpperCase();
//   if (cmd == "G") {
//     // TEST(0);
//     while (1) {
//       MOV_Radius(0.3, 0, 0, 0.3);
//     }
//   } else if (cmd == "S") {
//     STOP_ALL();
//   } else if (cmd == "H") {
//     HOME_ALL();
//   }
// }