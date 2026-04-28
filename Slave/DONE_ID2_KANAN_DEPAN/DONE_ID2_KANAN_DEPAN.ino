#define _VAR_GLOBAL_IMPLEMENTATION_
#include <Arduino.h>
#include <STM32FreeRTOS.h>

#include "VAR_GLOBAL.h"
#include "Steering.h"
#include "Driving.h"
#include "Communication.h"

// ==================== AUTO RUN PLAN ====================
struct Movement {
  float angle;
  float rpm;
  int duration;
};

Movement movements[] = {
  // { 0, 0, 5000 },
  // { 10, 0, 2000 },
  // { 350, 0, 2000 },
  // { 20, 0, 2000 },
  // { 340, 0, 2000 },
  // { 30, 0, 2000 },
  // { 330, 0, 2000 },
  // { 90, 0, 2000 },
  // { 270, 0, 2000 },
  // { 0, 50, 2000 },
  // { 180, 50, 2000 },
  // { 10, 50, 2000 },
  // { 350, 50, 2000 },
  // { 20, 50, 2000 },
  // { 340, 50, 2000 },
  // { 30, 50, 2000 },
  // { 330, 50, 2000 },
  // { 90, 50, 2000 },
  // { 270, 50, 2000 },
};

int totalSteps = sizeof(movements) / sizeof(movements[0]);
int currentStep = 0;
bool autoRepeat = true;
bool autoRunEnabled = false;

// ==================== AUTO RUN TASK ====================
void autoRunTask(void *pvParameters) {
  SwerveCommand_t cmd;

  vTaskDelay(pdMS_TO_TICKS(2000));

  if (serialMutex != NULL && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.println("\nAUTO RUN STARTED!");
    xSemaphoreGive(serialMutex);
  }

  while (1) {
    if (autoRunEnabled) {
      cmd.angle = movements[currentStep].angle;
      cmd.rpm = movements[currentStep].rpm;
      cmd.stop_all = false;
      cmd.is_new_command = true;
      xQueueSend(steerCommandQueue, &cmd, 0);
      xQueueSend(driveCommandQueue, &cmd, 0);

      vTaskDelay(pdMS_TO_TICKS(movements[currentStep].duration));

      currentStep++;
      if (currentStep >= totalSteps) {
        if (autoRepeat) {
          currentStep = 0;
        } else {
          break;
        }
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  delay(1000);

  steerCommandQueue = xQueueCreate(10, sizeof(SwerveCommand_t));
  driveCommandQueue = xQueueCreate(10, sizeof(SwerveCommand_t));
  rs485CommandQueue = xQueueCreate(10, sizeof(SwerveCommand_t));
  serialMutex = xSemaphoreCreateMutex();

  pinMode(STEER_ENC_A, INPUT_PULLUP);
  pinMode(STEER_ENC_B, INPUT_PULLUP);
  pinMode(STEER_RPWM, OUTPUT);
  pinMode(STEER_LPWM, OUTPUT);

  pinMode(DRIVE_ENC_A, INPUT_PULLUP);
  pinMode(DRIVE_ENC_B, INPUT_PULLUP);
  pinMode(DRIVE_RPWM, OUTPUT);
  pinMode(DRIVE_LPWM, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  pinMode(HOMING_SENSOR_KIRI, INPUT);
  pinMode(HOMING_SENSOR_KANAN, INPUT);

  attachInterrupt(digitalPinToInterrupt(STEER_ENC_A), steer_updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEER_ENC_B), steer_updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_A), drive_encoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_B), drive_encoderInterrupt, CHANGE);

  steer_lastTime = micros();
  drive_lastPidTime = micros();
  drive_lastRpmTime = millis();

  if (serialMutex != NULL && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.println("\n╔════════════════════════════════════════════╗");
    Serial.println("║     SWERVE DRIVE CONTROL SYSTEM v9.0      ║");
    Serial.println("║     WITH RS485 ANTI-RESET FILTER           ║");
    Serial.println("╚════════════════════════════════════════════╝");
    xSemaphoreGive(serialMutex);
  }

  xTaskCreate(steeringTask, "Steering", 256, NULL, 3, &steeringTaskHandle);
  xTaskCreate(drivingTask, "Driving", 256, NULL, 3, &drivingTaskHandle);
  xTaskCreate(rs485ExecuteTask, "RS485_Exec", 256, NULL, 4, &rs485ExecuteTaskHandle);
  xTaskCreate(commandTask, "Command", 128, NULL, 2, &commandTaskHandle);
  xTaskCreate(rs485Task, "RS485_RX", 256, NULL, 3, NULL);
  xTaskCreate(displayTask, "Display", 128, NULL, 1, &displayTaskHandle);
  xTaskCreate(debugUsbSlaveTask, "DEBUG_USB", 256, NULL, 1, NULL);
  xTaskCreate(homingTask, "Homing", 256, NULL, 2, &homingTaskHandle);

  // if (autoRunEnabled) {
  //   xTaskCreate(autoRunTask, "AutoRun", 256, NULL, 1, NULL);
  // }

  vTaskStartScheduler();
}

void loop() {
  // Empty
}