#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "VAR_GLOBAL.h"
#include "Steering.h"
#include "Driving.h"

// Function prototypes
void commandTask(void *pvParameters);
void displayTask(void *pvParameters);
void rs485Task(void *pvParameters);
void guiOutputTask(void *pvParameters);
void parseCommand(String cmd);
void parseRs485Command(String cmd);
void printHelp();
void printSystemStatus();
void executeSwerveCommand(float angle, float rpm);
void sendRs485Response(String response);
void sendGuiData();
void sendRs485RealtimeData();
void rs485RealtimeTask(void *pvParameters);

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
    } else {
      Serial.println(" | Driving: OFF");
    }
    xSemaphoreGive(serialMutex);
  }
}

void parseCommand(String cmd) {
  // ========== HOMING COMMANDS ==========
  if (cmd == "HOME" || cmd == "HOMING" || cmd == "RESET" || cmd == "RE" || cmd == "STATUS" || cmd == "ST") {
    homing_handleCommand(cmd);
    return;
  }
  
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
  
  int firstColon = cmd.indexOf(':');
  int secondColon = cmd.indexOf(':', firstColon + 1);
  
  if (firstColon > 0 && secondColon > firstColon) {
    float angle = cmd.substring(firstColon + 1, secondColon).toFloat();
    float rpm = cmd.substring(secondColon + 1).toFloat();
    executeSwerveCommand(angle, rpm);
    return;
  }
  
  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("❌ Command tidak dikenal! Gunakan HELP");
    xSemaphoreGive(serialMutex);
  }
}

void sendRs485Response(String response) {
  Serial1.println(response);
}

void parseRs485Command(String cmd) {
  int firstColon = cmd.indexOf(':');
  int secondColon = cmd.indexOf(':', firstColon + 1);
  
  if (firstColon > 0 && secondColon > firstColon) {
    int slaveId = cmd.substring(0, firstColon).toInt();
    
    if (slaveId == RS485_SLAVE_ID) {
      float angle = cmd.substring(firstColon + 1, secondColon).toFloat();
      float rpm = cmd.substring(secondColon + 1).toFloat();
      
      executeSwerveCommand(angle, rpm);
      sendRs485Response("OK:" + String(RS485_SLAVE_ID) + ":A" + String(angle) + ":R" + String(rpm));
      
      if (serialMutex != NULL) {
        xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.print("📡 RS485 [ID:");
        Serial.print(slaveId);
        Serial.print("] A");
        Serial.print(angle);
        Serial.print(" R");
        Serial.println(rpm);
        xSemaphoreGive(serialMutex);
      }
    }
  } else {
    sendRs485Response("ERROR:INVALID_FORMAT");
  }
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

void guiOutputTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    sendGuiData();
  }
}

void printHelp() {
  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    
    Serial.println("\n╔════════════════════════════════════════════╗");
    Serial.println("║            HELP - SWERVE DRIVE             ║");
    Serial.println("╚════════════════════════════════════════════╝");
    
    Serial.println("\n📝 FORMAT INPUT (USB Serial):");
    Serial.println("  A[angle]R[rpm]  - Set steering dan driving");
    Serial.println("  Contoh: A45R20   -> Steering 45°, Maju 20 RPM");
    Serial.println("          A225R20  -> Steering 225°, Mundur 20 RPM");
    Serial.println("          A90R0    -> Steering 90° saja");
    Serial.println("          STOP     -> Emergency stop");
    Serial.println("          1:45:20  -> Format RS485 via USB");
    
    Serial.println("\n🏠 HOMING COMMANDS:");
    Serial.println("  HOME/HOMING - Mulai auto homing");
    Serial.println("  RESET/RE    - Reset encoder ke 0°");
    Serial.println("  STATUS/ST   - Lihat status homing");
    
    Serial.println("\n📡 OUTPUT FORMAT (ke GUI):");
    Serial.println("  ID:angle,rpm,pwm_pos,pwm_spd,ready");
    Serial.println("  Contoh: 1:45.5,30.2,50,120,1");
    
    Serial.println("\n⚙️ LOGIKA ARAH BERDASARKAN SUDUT:");
    Serial.println("  MAJU   : 0° - 90° dan 270° - 360°");
    Serial.println("  MUNDUR : 90° - 270°");
    
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
    Serial.print("  Posisi: ");
    Serial.print(steerAngleDeg, 1);
    Serial.print("° | Target: ");
    Serial.print(steerRawTargetAngle, 1);
    Serial.print("° | PWM: ");
    Serial.println(steer_currentOutput, 0);
    
    Serial.println("\n⚡ DRIVING:");
    Serial.print("  RPM: ");
    Serial.print(drive_wheelRpm, 1);
    Serial.print(" | Target: ");
    Serial.print(drive_wheelRpmTarget_requested, 1);
    Serial.print(" | PWM: ");
    Serial.println(drive_pwmOutput);
    
    Serial.print("  Status: ");
    if (steer_targetReached) Serial.print("STEER READY | ");
    else Serial.print("STEER MOVING | ");
    
    if (driving_active) Serial.println("DRIVE ACTIVE");
    else Serial.println("DRIVE IDLE");
    
    Serial.println("\n📤 OUTPUT KE GUI:");
    Serial.print("  Format: ");
    Serial.println("ID:angle,rpm,pwm_pos,pwm_spd,ready");
    
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
  }
}

void rs485Task(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);
  
  Serial1.begin(115200);
  
  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("📡 RS485 Started on Serial1");
    Serial.print("   Slave ID: ");
    Serial.println(RS485_SLAVE_ID);
    xSemaphoreGive(serialMutex);
  }
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    while (Serial1.available() > 0) {
      char c = Serial1.read();
      
      if (c == '\n') {
        if (rs485BufferIndex > 0) {
          rs485Buffer[rs485BufferIndex] = '\0';
          String command = String(rs485Buffer);
          command.trim();
          parseRs485Command(command);
          rs485BufferIndex = 0;
          memset(rs485Buffer, 0, sizeof(rs485Buffer));
        }
      } else if (c != '\r' && rs485BufferIndex < sizeof(rs485Buffer) - 1) {
        rs485Buffer[rs485BufferIndex++] = c;
      }
    }
  }
}

void displayTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(500);
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    if (serialMutex != NULL && xSemaphoreTake(serialMutex, 0) == pdTRUE) {
      static unsigned long lastBlink = 0;
      if (millis() - lastBlink > 500) {
        lastBlink = millis();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      }
      xSemaphoreGive(serialMutex);
    }
  }
}

#pragma endregion
#endif