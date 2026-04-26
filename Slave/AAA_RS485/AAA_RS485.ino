#define _VAR_GLOBAL_IMPLEMENTATION_  // INI BARIS PERTAMA - definisikan variabel
#include <Arduino.h>
#include <STM32FreeRTOS.h>

#include "VAR_GLOBAL.h"
#include "Steering.h"
#include "Driving.h"
#include "Communication.h"

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Buat queue dan mutex
  steerCommandQueue = xQueueCreate(5, sizeof(SwerveCommand_t));
  driveCommandQueue = xQueueCreate(5, sizeof(SwerveCommand_t));
  serialMutex = xSemaphoreCreateMutex();
  
  // Setup pins
  pinMode(STEER_ENC_A, INPUT_PULLUP);
  pinMode(STEER_ENC_B, INPUT_PULLUP);
  pinMode(STEER_RPWM, OUTPUT);
  pinMode(STEER_LPWM, OUTPUT);
  
  pinMode(DRIVE_ENC_A, INPUT_PULLUP);
  pinMode(DRIVE_ENC_B, INPUT_PULLUP);
  pinMode(DRIVE_RPWM, OUTPUT);
  pinMode(DRIVE_LPWM, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Interrupts
  attachInterrupt(digitalPinToInterrupt(STEER_ENC_A), steer_updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEER_ENC_B), steer_updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_A), drive_encoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_B), drive_encoderInterrupt, CHANGE);
  
  // Init timing
  steer_lastTime = micros();
  drive_lastPidTime = micros();
  drive_lastRpmTime = millis();
  
  // Print welcome
  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("\n╔════════════════════════════════════════════╗");
    Serial.println("║     SWERVE DRIVE CONTROL SYSTEM v8.0      ║");
    Serial.println("╚════════════════════════════════════════════╝");
    xSemaphoreGive(serialMutex);
  }
  
  // Create tasks
  xTaskCreate(steeringTask, "Steering", 256, NULL, 2, &steeringTaskHandle);
  xTaskCreate(drivingTask, "Driving", 256, NULL, 2, &drivingTaskHandle);
  xTaskCreate(commandTask, "Command", 128, NULL, 3, &commandTaskHandle);
  xTaskCreate(displayTask, "Display", 128, NULL, 1, &displayTaskHandle);
  xTaskCreate(rs485Task, "RS485", 256, NULL, 3, NULL);  // <-- TAMBAH INI
  
  vTaskStartScheduler();
}

void loop() {
  // Empty
}