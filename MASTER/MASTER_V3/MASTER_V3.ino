#include <Wire.h>
#include "Var.h"
#include "Sensor_Odometry.h"
#include "SwerveDrive.h"
#include "PID.h"
#include "DisplayControl.h"
#include "Plan.h"

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
  if (W && D) {
    Serial.println("MAJU KANAN");
    SwerveDrive(0.3, -0.3, 0, 30);
  } else if (W && A) {
    Serial.println("MAJU Kiri");
    SwerveDrive(0.3, 0.3, 0, 30);
  } else if (S && D) {
    Serial.println("Mundur KANAN");
    SwerveDrive(-0.3, -0.3, 0, 30);
  } else if (S && A) {
    Serial.println("Mundur Kiri");
    SwerveDrive(-0.3, 0.3, 0, 30);
  } else if (W) {
    Serial.println("MAJU");
    // SwerveDrive(0.3, 0, 0, 30);
    PID(F_GY, 50, 180, 100);
  } else if (S) {
    Serial.println("Mundur");
    SwerveDrive(-0.3, 0, 0, 30);
  } else if (A) {
    Serial.println("kiri");
    SwerveDrive(0, 0.3, 0, 30);
  } else if (D) {
    Serial.println("kanan");
    SwerveDrive(0, -0.3, 0, 30);
  } else if (J) {
    SwerveDrive(0, 0, 0.3, 30);
    Serial.println("CCW");
  } else if (K) {
    SwerveDrive(0, 0, -0.3, 30);
    Serial.println("CW");
  } else if (STOP) {
    STOP_ALL();
    Serial.println("Stop");
  } else if (P) {
    STOP_ALL();
    Serial.println("Stop");
  } else if (H) {
    HOME_ALL();
    Serial.println("HOMING");
  } else if (P1) {
    ModeBuzzer(TULULIT);
  } else if (P2) {
    ModeBuzzer(Armed);
  }
}