#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "VAR_GLOBAL.h"
#include "Steering.h"
#include "Driving.h"

// Function prototypes
void commandTask(void *pvParameters);
void displayTask(void *pvParameters);
void rs485Task(void *pvParameters);
void rs485ExecuteTask(void *pvParameters);
void debugUsbSlaveTask(void *pvParameters);
void parseCommand(String cmd);
void parseRs485Command(String cmd);
void printSystemStatus();
void executeSwerveCommand(float angle, float rpm);
void executeSwerveCommandRS485(float angle, float rpm);
void debugUsbSlave();

// ==================== COMMUNICATION IMPLEMENTATIONS ====================
#pragma region Communication_Implementations

void executeSwerveCommand(float angle, float rpm) {
  SwerveCommand_t cmd;
  cmd.angle = angle;
  cmd.rpm = rpm;
  cmd.stop_all = false;
  cmd.is_new_command = true;  // USB selalu command baru
  
  xQueueSend(steerCommandQueue, &cmd, 0);
  xQueueSend(driveCommandQueue, &cmd, 0);
  
  if (serialMutex != NULL && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    Serial.print("USB CMD: A");
    Serial.print(angle, 1);
    Serial.print(" R");
    Serial.println(rpm, 1);
    xSemaphoreGive(serialMutex);
  }
}

// FUNGSI KRITIS: Filter perintah RS485 yang berulang
void executeSwerveCommandRS485(float angle, float rpm) {
  unsigned long now = millis();
  
  // Cek apakah command SAMA dengan sebelumnya dan dalam waktu dekat
  if (angle == lastRs485Angle && rpm == lastRs485Rpm && 
      (now - lastRs485CmdTime) < CMD_TIMEOUT_MS) {
    // Command sama dalam waktu dekat, IGNORE (jangan kirim lagi)
    return;
  }
  
  // Update last command tracking
  lastRs485Angle = angle;
  lastRs485Rpm = rpm;
  lastRs485CmdTime = now;
  
  // Kirim command baru
  SwerveCommand_t cmd;
  cmd.angle = angle;
  cmd.rpm = rpm;
  cmd.stop_all = false;
  cmd.is_new_command = true;
  
  // Kirim ke queue RS485 untuk diproses
  xQueueSend(rs485CommandQueue, &cmd, 0);
  
  if (serialMutex != NULL && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    Serial.print("RS485 CMD: A");
    Serial.print(angle, 1);
    Serial.print(" R");
    Serial.println(rpm, 1);
    xSemaphoreGive(serialMutex);
  }
}

void rs485ExecuteTask(void *pvParameters) {
  SwerveCommand_t cmd;
  
  while(1) {
    // Tunggu command baru dari queue
    if (xQueueReceive(rs485CommandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      // Kirim ke steering dan driving queues
      xQueueSend(steerCommandQueue, &cmd, 0);
      xQueueSend(driveCommandQueue, &cmd, 0);
    }
  }
}

void parseCommand(String cmd) {
  // ========== HOMING COMMANDS ==========
  if (cmd == "HOME" || cmd == "HOMING" || cmd == "RESET" || cmd == "RE" || cmd == "STATUS" || cmd == "ST") {
    homing_handleCommand(cmd);
    return;
  }
  
  if (cmd == "HELP" || cmd == "H") {
    if (serialMutex != NULL && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      Serial.println("Commands: A[angle]R[rpm] e.g. A45R20, STOP, STATUS, HOME");
      xSemaphoreGive(serialMutex);
    }
    return;
  }
  
  if (cmd == "STOP" || cmd == "S") {
    SwerveCommand_t stopCmd;
    stopCmd.stop_all = true;
    stopCmd.is_new_command = true;
    xQueueSend(steerCommandQueue, &stopCmd, 0);
    xQueueSend(driveCommandQueue, &stopCmd, 0);
    
    if (serialMutex != NULL && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      Serial.println("STOP - SEMUA MOTOR");
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
  
  if (serialMutex != NULL && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    Serial.println("Command tidak dikenal! Gunakan HELP");
    xSemaphoreGive(serialMutex);
  }
}

void sendRs485Response(String response) {
  Serial1.println(response);
}

bool isIntegerString(const String &value) {
  if (value.length() == 0) return false;
  int start = 0;
  if (value[0] == '+' || value[0] == '-') {
    if (value.length() == 1) return false;
    start = 1;
  }
  for (int i = start; i < value.length(); i++) {
    char c = value[i];
    if (c < '0' || c > '9') return false;
  }
  return true;
}

bool isNumericString(const String &value) {
  if (value.length() == 0) return false;
  int start = 0;
  if (value[0] == '+' || value[0] == '-') {
    if (value.length() == 1) return false;
    start = 1;
  }
  bool hasDecimal = false;
  for (int i = start; i < value.length(); i++) {
    char c = value[i];
    if (c == '.') {
      if (hasDecimal) return false;
      hasDecimal = true;
      continue;
    }
    if (c < '0' || c > '9') return false;
  }
  return true;
}

void parseRs485Command(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  int firstColon = cmd.indexOf(':');
  if (firstColon <= 0) return;

  String idStr = cmd.substring(0, firstColon);
  idStr.trim();

  // ===== FORMAT ALL: BROADCAST =====
  if (idStr == "ALL" || idStr == "all") {
    String data = cmd.substring(firstColon + 1);
    int pos = 0;
    int wheelIndex = 0;

    while (pos < data.length() && wheelIndex < 4) {
      int nextColon = data.indexOf(':', pos);
      String wheelStr;

      if (nextColon == -1) {
        wheelStr = data.substring(pos);
      } else {
        wheelStr = data.substring(pos, nextColon);
      }
      wheelStr.trim();

      int comma = wheelStr.indexOf(',');
      if (comma > 0) {
        String angleStr = wheelStr.substring(0, comma);
        String rpmStr = wheelStr.substring(comma + 1);
        angleStr.trim();
        rpmStr.trim();
        float angle = angleStr.toFloat();
        float rpm = rpmStr.toFloat();

        if ((wheelIndex + 1) == RS485_SLAVE_ID) {
          executeSwerveCommandRS485(angle, rpm);
        }
      }

      wheelIndex++;
      if (nextColon == -1) break;
      pos = nextColon + 1;
    }
    return;
  }

  if (!isIntegerString(idStr)) return;
  int slaveId = idStr.toInt();
  if (slaveId != RS485_SLAVE_ID) return;

  String afterId = cmd.substring(firstColon + 1);
  afterId.trim();
  afterId.toUpperCase();

  if (afterId == "HOME" || afterId == "HOMING") {
    homing_handleCommand("HOME");
    sendRs485Response("OK:" + String(RS485_SLAVE_ID) + ":HOME");
    return;
  }

  if (afterId == "STOP") {
    SwerveCommand_t stopCmd;
    stopCmd.stop_all = true;
    stopCmd.is_new_command = true;
    xQueueSend(steerCommandQueue, &stopCmd, 0);
    xQueueSend(driveCommandQueue, &stopCmd, 0);
    
    // Reset tracking saat stop
    lastRs485Angle = -999.0;
    lastRs485Rpm = -999.0;
    
    sendRs485Response("OK:" + String(RS485_SLAVE_ID) + ":STOP");
    return;
  }

  int secondColon = cmd.indexOf(':', firstColon + 1);
  if (secondColon <= firstColon) return;

  String angleStr = cmd.substring(firstColon + 1, secondColon);
  String rpmStr = cmd.substring(secondColon + 1);
  angleStr.trim();
  rpmStr.trim();

  if (!isNumericString(angleStr) || !isNumericString(rpmStr)) return;

  float angle = angleStr.toFloat();
  float rpm = rpmStr.toFloat();

  executeSwerveCommandRS485(angle, rpm);
  sendRs485Response("OK:" + String(RS485_SLAVE_ID) + ":A" + String(angle) + ":R" + String(rpm));
}

void debugUsbSlave() {
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

void debugUsbSlaveTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    debugUsbSlave();
  }
}

void printHelp() {
  if (serialMutex != NULL && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    Serial.println("\nHELP - SWERVE DRIVE");
    Serial.println("Commands: A[angle]R[rpm], STOP, STATUS, HOME");
    xSemaphoreGive(serialMutex);
  }
}

void printSystemStatus() {
  if (serialMutex != NULL && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    Serial.println("\n=== STATUS ===");
    Serial.print("STEER: ");
    Serial.print(steerAngleDeg, 1);
    Serial.print(" -> ");
    Serial.print(steerRawTargetAngle, 1);
    Serial.print(" PWM:");
    Serial.println(steer_currentOutput, 0);
    
    Serial.print("DRIVE: ");
    Serial.print(drive_wheelRpm, 1);
    Serial.print(" -> ");
    Serial.print(drive_wheelRpmTarget_requested, 1);
    Serial.print(" PWM:");
    Serial.println(drive_pwmOutput);
    
    Serial.print("STATUS: ");
    if (steer_targetReached) Serial.print("STEER OK | ");
    else Serial.print("STEER MOV | ");
    
    if (driving_active) Serial.println("DRIVE ON");
    else Serial.println("DRIVE OFF");
    
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
  const TickType_t xFrequency = pdMS_TO_TICKS(5);  // 5ms cycle
  
  Serial1.begin(115200);
  
  if (serialMutex != NULL && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.println("RS485 Started on Serial1");
    Serial.print("Slave ID: ");
    Serial.println(RS485_SLAVE_ID);
    xSemaphoreGive(serialMutex);
  }
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // Baca semua data yang tersedia
    while (Serial1.available() > 0) {
      char c = Serial1.read();
      
      if (c == '\n') {
        if (rs485BufferIndex > 0) {
          rs485Buffer[rs485BufferIndex] = '\0';
          String command = String(rs485Buffer);
          command.trim();
          
          // Parse dan eksekusi (dengan filter di dalamnya)
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