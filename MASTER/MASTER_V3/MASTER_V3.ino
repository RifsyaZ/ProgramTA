#include "Var.h"
#include "Sensor_Odometry.h"
#include "SwerveDrive.h"
#include "Plan.h"
#include "I2C_Comm.h"

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);   // Serial Monitor
  Serial1.begin(115200);  // RS485 ke motor
  Serial2.begin(115200);  // Feedback dari ID1
  Serial3.begin(115200);  // Feedback dari ID2
  Serial4.begin(115200);  // Feedback dari ID3
  Serial5.begin(115200);  // komunikasi MCU MPU
  Serial6.begin(115200);  // komunikasi KE ID 4

  I2C_Master_Init();  // Initialize I2C Master
  delay(100);
  I2C_ScanBus();
  Serial.println("[MASTER] I2C Ready!");

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
  
  // ===== BACA COMMAND DARI ESP32 =====
  char ble_cmd = I2C_ReadBLECommand();
  if (ble_cmd != '0') {
    Serial.print("[I2C RX] BLE Command: ");
    Serial.println(ble_cmd);
    
    if (ble_cmd == 'F') {
      Serial.println("MAJU");
    }
    else if (ble_cmd == 'B') {
      Serial.println("MUNDUR");
    }
    else if (ble_cmd == 'L') {
      Serial.println("KIRI");
    }
    else if (ble_cmd == 'R') {
      Serial.println("KANAN");
    }
    else if (ble_cmd == 'S') {
      Serial.println("STOP");
    }
  }
  
  // ===== KIRIM ODOMETRY KE ESP32 =====
  I2C_SendOdometry(yaw, 
                   Fedback_Angle[0], Fedback_Angle[1], 
                   Fedback_Angle[2], Fedback_Angle[3],
                   (int)Fedback_Pulse[0], (int)Fedback_Pulse[1],
                   (int)Fedback_Pulse[2], (int)Fedback_Pulse[3]);
  
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
}