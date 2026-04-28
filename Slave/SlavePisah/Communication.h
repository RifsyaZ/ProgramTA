#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "VAR_GLOBAL.h"
#include "Steering.h"
#include "Driving.h"

// Function prototypes
void commandTask(void *pvParameters);
void displayTask(void *pvParameters);
void parseCommand(String cmd);
void printHelp();
void printSystemStatus();
void executeSwerveCommand(float angle, float rpm);
void sendRs485RealtimeData();
void sendGuiData();
void rs485RealtimeTask(void *pvParameters);
void guiOutputTask(void *pvParameters);

// RS485 (jika diperlukan di masa depan)
// #define RS485_DE_RE PCx
// HardwareSerial RS485Serial(Serial2);

// ==================== COMMUNICATION IMPLEMENTATIONS ====================
#pragma region Communication_Implementations

void executeSwerveCommand(float angle, float rpm) {
  SwerveCommand_t cmd;
  cmd.angle = angle;
  cmd.rpm = rpm;
  cmd.stop_all = false;
  
  xQueueSend(steerCommandQueue, &cmd, 0);
  xQueueSend(driveCommandQueue, &cmd, 0);
  
  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("\n🔄 EKSEKUSI PERINTAH:");
    Serial.print("   Steering: ");
    Serial.print(angle, 1);
    Serial.print("°");
    
    if (rpm != 0) {
      bool forward = drive_determineDirection(angle);
      Serial.print(" | Driving: ");
      Serial.print(rpm, 1);
      Serial.print(" RPM ");
      Serial.print(forward ? "MAJU" : "MUNDUR");
      Serial.print(" (sudut ");
      Serial.print(angle, 1);
      Serial.println("°)");
    } else {
      Serial.println(" | Driving: OFF");
    }
    xSemaphoreGive(serialMutex);
  }
}

void parseCommand(String cmd) {
  if (cmd == "HELP" || cmd == "H") {
    printHelp();
    return;
  }
  
  if (cmd == "STOP" || cmd == "S") {
    SwerveCommand_t stopCmd;
    stopCmd.stop_all = true;
    xQueueSend(steerCommandQueue, &stopCmd, 0);
    xQueueSend(driveCommandQueue, &stopCmd, 0);
    
    if (serialMutex != NULL) {
      xSemaphoreTake(serialMutex, portMAX_DELAY);
      Serial.println("⏹️ EMERGENCY STOP - SEMUA MOTOR");
      xSemaphoreGive(serialMutex);
    }
    return;
  }
  
  if (cmd == "STATUS" || cmd == "?") {
    printSystemStatus();
    return;
  }
  
  int aIndex = cmd.indexOf('A');
  int rIndex = cmd.indexOf('R');
  
  if (aIndex >= 0 && rIndex > aIndex) {
    String angleStr = cmd.substring(aIndex + 1, rIndex);
    String rpmStr = cmd.substring(rIndex + 1);
    
    float angle = angleStr.toFloat();
    float rpm = rpmStr.toFloat();
    
    executeSwerveCommand(angle, rpm);
    return;
  }
  
  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("❌ Command tidak dikenal! Gunakan HELP");
    xSemaphoreGive(serialMutex);
  }
}

void printHelp() {
  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    
    Serial.println("\n╔════════════════════════════════════════════╗");
    Serial.println("║            HELP - SWERVE DRIVE             ║");
    Serial.println("╚════════════════════════════════════════════╝");
    
    Serial.println("\n📝 FORMAT INPUT:");
    Serial.println("  A[angle]R[rpm]  - Set steering dan driving");
    Serial.println("  Contoh: A45R20   -> Steering 45°, Maju 20 RPM");
    Serial.println("          A225R20  -> Steering 225°, Mundur 20 RPM");
    Serial.println("          A90R0    -> Steering 90° saja");
    Serial.println("          STOP     -> Emergency stop");
    
    Serial.println("\n⚙️ LOGIKA ARAH BERDASARKAN SUDUT:");
    Serial.println("  MAJU   : 0° - 90° dan 270° - 360°");
    Serial.println("  MUNDUR : 90° - 270°");
    
    Serial.println("\n⚙️ MODE OPERASI:");
    Serial.println("  • Steering dan Driving bekerja BERSAMAAN");
    Serial.println("  • Tidak ada penundaan/delay");
    
    Serial.println("\n📋 PERINTAH LAIN:");
    Serial.println("  STATUS/? - Tampilkan status");
    Serial.println("  HELP / H - Tampilkan help");
    
    Serial.println("\n============================================\n");
    
    xSemaphoreGive(serialMutex);
  }
}

void printSystemStatus() {
  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    
    Serial.println("\n╔════════════════════════════════════════════╗");
    Serial.println("║            SYSTEM STATUS                   ║");
    Serial.println("╚════════════════════════════════════════════╝");
    
    Serial.println("\n🎯 STEERING:");
    Serial.print("  Rate: 2ms | Posisi: ");
    Serial.print(steerAngleDeg, 1);
    Serial.print("° | Target: ");
    Serial.print(steerTargetAngle, 1);
    Serial.print("° (");
    Serial.print(steerRawTargetAngle, 1);
    Serial.println("°)");
    
    float steerErr = steer_getCurrentError();
    Serial.print("  Error: ");
    Serial.print(steerErr, 2);
    Serial.print("° | Output: ");
    Serial.println(steer_currentOutput, 1);
    
    Serial.print("  Status: ");
    if (!steering_active) Serial.println("IDLE");
    else if (steer_targetReached) Serial.println("ON TARGET");
    else Serial.println("MOVING");
    
    Serial.println("\n⚡ DRIVING:");
    Serial.print("  Rate: 20ms | Request: ");
    Serial.print(abs(drive_wheelRpmTarget_requested), 1);
    Serial.print(" RPM ");
    Serial.println(drive_directionForward ? "MAJU" : "MUNDUR");
    
    Serial.print("  Aktual: ");
    Serial.print(drive_wheelRpm, 1);
    Serial.print(" RPM | PWM: ");
    Serial.println(drive_pwmOutput);
    
    Serial.print("  Status: ");
    if (!driving_active) Serial.println("IDLE");
    else Serial.println("AKTIF");
    
    Serial.println("\n============================================\n");
    
    xSemaphoreGive(serialMutex);
  }
}

void commandTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    while (Serial.available() > 0) {
      char c = Serial.read();
      
      if (c == '\n') {
        if (bufferIndex > 0) {
          inputBuffer[bufferIndex] = '\0';
          String command = String(inputBuffer);
          command.trim();
          command.toUpperCase();
          parseCommand(command);
          bufferIndex = 0;
          memset(inputBuffer, 0, sizeof(inputBuffer));
        }
      } else if (c != '\r' && bufferIndex < sizeof(inputBuffer) - 1) {
        inputBuffer[bufferIndex++] = c;
      }
    }
    
    // Kode untuk RS485 jika diperlukan di masa depan
    // if (RS485Serial.available()) {
    //   // Handle RS485 communication
    // }
  }
}

void displayTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(500);
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    if (serialMutex != NULL && xSemaphoreTake(serialMutex, 0) == pdTRUE) {
      Serial.println("----------------------------------------");
      
      Serial.print("STEERING: ");
      Serial.print(steerAngleDeg, 1);
      Serial.print("° → ");
      Serial.print(steerTargetAngle, 1);
      Serial.print("° (");
      Serial.print(steerRawTargetAngle, 1);
      Serial.print("°)");
      
      float steerError = steer_getCurrentError();
      Serial.print(" | Err: ");
      Serial.print(steerError, 2);
      
      if (steer_targetReached) {
        Serial.print(" | 🟢 ON TARGET");
      } else if (steering_active) {
        Serial.print(" | 🔴 MOVING");
      } else {
        Serial.print(" | ⚪ IDLE");
      }
      Serial.println();
      
      Serial.print("DRIVING : ");
      if (!driving_active) {
        Serial.print("💤 IDLE");
      } else if (drive_directionForward) {
        Serial.print("🚗 MAJU >>>");
      } else {
        Serial.print("🚗 MUNDUR <<<");
      }
      
      Serial.print(" | RPM: ");
      Serial.print(drive_wheelRpm, 1);
      Serial.print(" / ");
      Serial.print(drive_wheelRpmTarget_requested, 1);
      
      if (driving_active) {
        float driveError = drive_wheelRpmTarget_requested - drive_wheelRpm;
        Serial.print(" | Err: ");
        Serial.print(driveError, 1);
        Serial.print(" | PWM: ");
        Serial.print(drive_pwmOutput);
      }
      Serial.println();
      
      if (steering_active || driving_active) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      } else {
        digitalWrite(LED_PIN, HIGH);
      }
      
      xSemaphoreGive(serialMutex);
    }
  }
}

void sendRs485RealtimeData() {
  int ready = steer_targetReached ? 1 : 0;
  int pwm_pos = (int)steer_currentOutput;
  int pwm_spd = drive_pwmOutput;
  
  String output = String(RS485_SLAVE_ID) + ":" + 
                  String(steerAngleDeg, 1) + "," +
                  String(drive_wheelRpm, 1) + "," +
                  String(pwm_pos) + "," +
                  String(pwm_spd) + "," +
                  String(ready);
  
  Serial1.println(output);
}

void sendGuiData() {
  int ready = steer_targetReached ? 1 : 0;
  int pwm_pos = (int)steer_currentOutput;
  int pwm_spd = drive_pwmOutput;
  
  String output = String(RS485_SLAVE_ID) + ":" + 
                  String(steerAngleDeg, 1) + "," +
                  String(drive_wheelRpm, 1) + "," +
                  String(pwm_pos) + "," +
                  String(pwm_spd) + "," +
                  String(ready);
  
  Serial.println(output);
}

void rs485RealtimeTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  
  vTaskDelay(pdMS_TO_TICKS(500));
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    sendRs485RealtimeData();
  }
}

void guiOutputTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    sendGuiData();
  }
}

#pragma endregion
#endif