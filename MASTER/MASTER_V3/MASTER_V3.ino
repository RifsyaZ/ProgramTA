#include <Wire.h>
#include "Var.h"
#include "Sensor_Odometry.h"
#include "SwerveDrive.h"
#include "Plan.h"
#include "DisplayControl.h"

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);  // Serial Monitor
  Wire.begin();
  Serial1.begin(115200);  // RS485 ke motor
  Serial2.begin(115200);  // Feedback dari ID1
  Serial3.begin(115200);  // Feedback dari ID2
  Serial4.begin(115200);  // Feedback dari ID3
  Serial5.begin(115200);  // komunikasi MCU MPU
  Serial6.begin(115200);  // komunikasi KE ID 4

  pinMode(BUZZER_PIN, OUTPUT);
  ModeBuzzer(TULULIT);
  ModeBuzzer(TET_TET);
  while (1) {
    MPU1();
    if (yaw != 0) {
      MPU1();
      ModeBuzzer(MPU_Ready);
      break;
    } else {
      MPU1();
      Serial.println("MPU Belum Ready!!!");
      ModeBuzzer(MPU_BelumReady);
    }
  }
  ModeBuzzer(Armed);
}

// ==================== LOOP ====================
void loop() {
  MPU1();
  fedback();
  Debug_odometry();
  CommunicationESP();
  if (isForward && isRight) {
    Serial.println("MAJU KANAN");
  }  else if (isForward && isLeft) {
    Serial.println("MAJU Kiri");
  }
}

// if (Serial.available()) {
//   String cmd = Serial.readStringUntil('\n');
//   cmd.trim();
//   if (cmd.length() == 0) return;

//   cmd.toUpperCase();
//   if (cmd == "G") {
//     TEST(0);
//   } else if (cmd == "S") {
//     STOP_ALL();
//   } else if (cmd == "H") {
//     HOME_ALL();
//   }
// }