/**
 * SWERVE DRIVE CONTROL SYSTEM - PARALLEL EXECUTION
 * Steering dan Driving bekerja BERSAMAAN
 * 
 * Input: A[angle]R[rpm]  Contoh: A90R50
 * 
 * HARDWARE:
 * - Steering Motor: BTS7960 (RPWM=PB8, LPWM=PB9)
 * - Driving Motor: BTS7960 (RPWM=PA7, LPWM=PB10)
 */

#include <Arduino.h>
#include <STM32FreeRTOS.h>

// Include semua header
#include "VAR_GLOBAL.h"
#include "Steering.h"
#include "Driving.h"
#include "Communication.h"

// ==================== DEFINISI VARIABEL GLOBAL ====================
// Steering constants
const float STEER_MIN_ANGLE = -90.0;
const float STEER_MAX_ANGLE = 90.0;
const float STEER_TOLERANCE = 0.5;

float steer_Kp_normal = 2.5;
float steer_Ki_normal = 0.0;
float steer_Kd_normal = 1.2;

float steer_Kp_dekat = 0.5;
float steer_Ki_dekat = 0.0;
float steer_Kd_dekat = 0.1;

// Driving constants
const float DRIVE_PULSE_PER_REV = 14.0;
const float DRIVE_GEARBOX_RATIO = 6.77;
const float DRIVE_WHEEL_DIAMETER_MM = 82.0;
const float DRIVE_MAX_WHEEL_RPM = 800.0;

float drive_Kp = 0.05;
float drive_Ki = 0.5;
float drive_Kd = 0.001;

// Steering variables
volatile int32_t steerEncoderCount = 0;
int steerLastEncoded = 0;
float steerAngleDeg = 0;
float steerTargetAngle = 0;
float steerRawTargetAngle = 0;
float steerTargetSpeed = 100;
float steer_lastError = 0;
float steer_integral = 0;
float steer_prevError = 0;
float steer_currentOutput = 0;
unsigned long steer_lastTime = 0;
bool steer_targetReached = false;
bool steer_isLandingMode = false;
unsigned long steer_landingStartTime = 0;
float steer_lastErrorForLanding = 0;

// Driving variables
volatile int32_t driveEncoderCount = 0;
volatile uint32_t drive_lastInterruptTime = 0;
int32_t drive_lastPosition = 0;
uint32_t drive_lastRpmTime = 0;
float drive_motorRpm = 0.0;
float drive_wheelRpm = 0.0;
float drive_wheelRpmTarget = 0.0;
float drive_wheelRpmTarget_requested = 0.0;
float drive_motorRpmTarget = 0.0;
float drive_rpmFiltered = 0;
const float drive_filterAlpha = 0.2;
bool drive_directionForward = true;
float drive_requestedRpm = 0;
float drive_integral = 0;
float drive_prevError = 0;
uint32_t drive_lastPidTime = 0;
float drive_pidOutput = 0;
int drive_pwmOutput = 0;
bool drive_motorRunning = false;
bool drive_direction = true;
bool drive_encoderReverse = false;

// System variables
bool steering_active = false;
bool driving_active = false;
unsigned long command_start_time = 0;
float requested_angle = 0;
float requested_rpm = 0;

// Queue handles
QueueHandle_t steerCommandQueue;
QueueHandle_t driveCommandQueue;

// Task handles
TaskHandle_t steeringTaskHandle;
TaskHandle_t drivingTaskHandle;
TaskHandle_t commandTaskHandle;
TaskHandle_t displayTaskHandle;

SemaphoreHandle_t serialMutex;

// Buffers
const int STEER_NUM_READINGS = 3;
float steer_errorHistory[3] = {0};
int steer_readIndex = 0;
float steer_errorTotal = 0;

const float DRIVE_INTEGRAL_MAX = 500.0;
const float DRIVE_INTEGRAL_MIN = -500.0;
const int DRIVE_PWM_MIN = 15;
const int DRIVE_PWM_MAX = 255;

char inputBuffer[50];
int bufferIndex = 0;

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  steerCommandQueue = xQueueCreate(5, sizeof(SwerveCommand_t));
  driveCommandQueue = xQueueCreate(5, sizeof(SwerveCommand_t));
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
  
  attachInterrupt(digitalPinToInterrupt(STEER_ENC_A), steer_updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEER_ENC_B), steer_updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_A), drive_encoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_B), drive_encoderInterrupt, CHANGE);
  
  steerEncoderCount = 0;
  steerAngleDeg = 0;
  steer_lastTime = micros();
  drive_lastPidTime = micros();
  drive_lastRpmTime = millis();
  
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║     SWERVE DRIVE CONTROL SYSTEM v8.0      ║");
  Serial.println("║          BTS7960 DRIVER                    ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\n✅ STEERING: BTS7960 (RPWM=PB8, LPWM=PB9)");
  Serial.println("✅ DRIVING: BTS7960 (RPWM=PA7, LPWM=PB10)");
  Serial.println("\n📝 FORMAT INPUT: A[angle]R[rpm]");
  Serial.println("   Contoh: A45R20  -> Steering 45°, Maju 20 RPM");
  Serial.println("           A225R20 -> Steering 225°, Mundur 20 RPM");
  Serial.println("==============================================\n");
  xSemaphoreGive(serialMutex);
  
  xTaskCreate(steeringTask, "Steering Control", 256, NULL, 2, &steeringTaskHandle);
  xTaskCreate(drivingTask, "Driving Control", 256, NULL, 2, &drivingTaskHandle);
  xTaskCreate(commandTask, "Command Parser", 128, NULL, 3, &commandTaskHandle);
  xTaskCreate(displayTask, "Display", 128, NULL, 1, &displayTaskHandle);
  
  vTaskStartScheduler();
  
  while(1);
}

void loop() {
  // Empty - RTOS handles everything
}