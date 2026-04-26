/**
 * SWERVE DRIVE CONTROL SYSTEM - PARALLEL EXECUTION
 */

#include <Arduino.h>
#include <STM32FreeRTOS.h>

#include "VAR_GLOBAL.h"
#include "Steering.h"
#include "Driving.h"
#include "Communication.h"
#include "Homing.h"

// ==================== DEFINISI VARIABEL GLOBAL ====================
const float STEER_MIN_ANGLE = -90.0;
const float STEER_MAX_ANGLE = 90.0;
const float STEER_TOLERANCE = 0.5;

float steer_Kp_normal = 2.5;
float steer_Ki_normal = 0.0;
float steer_Kd_normal = 1.2;

float steer_Kp_dekat = 0.5;
float steer_Ki_dekat = 0.0;
float steer_Kd_dekat = 0.1;

const float DRIVE_PULSE_PER_REV = 14.0;
const float DRIVE_GEARBOX_RATIO = 6.77;
const float DRIVE_WHEEL_DIAMETER_MM = 82.0;
const float DRIVE_MAX_WHEEL_RPM = 800.0;

float drive_Kp = 0.05;
float drive_Ki = 0.5;
float drive_Kd = 0.001;

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

bool steering_active = false;
bool driving_active = false;
unsigned long command_start_time = 0;
float requested_angle = 0;
float requested_rpm = 0;

QueueHandle_t steerCommandQueue;
QueueHandle_t driveCommandQueue;

TaskHandle_t steeringTaskHandle;
TaskHandle_t drivingTaskHandle;
TaskHandle_t commandTaskHandle;
TaskHandle_t displayTaskHandle;

SemaphoreHandle_t serialMutex;

const int STEER_NUM_READINGS = 3;
float steer_errorHistory[3] = { 0 };
int steer_readIndex = 0;
float steer_errorTotal = 0;

const float DRIVE_INTEGRAL_MAX = 500.0;
const float DRIVE_INTEGRAL_MIN = -500.0;
const int DRIVE_PWM_MIN = 15;
const int DRIVE_PWM_MAX = 255;

char inputBuffer[50];
int bufferIndex = 0;

// HOMING VARIABLES
HomingState homingState = HOMING_IDLE;
bool homingComplete = false;
unsigned long homingStartTime = 0;
int lastLeftValue = 0;
int lastRightValue = 0;

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  
  // TUNGGU SERIAL READY
  while (!Serial && millis() < 3000);
  
  delay(1000);
  
  Serial.println("\n\n==============================================");
  Serial.println("SWERVE DRIVE SYSTEM BOOTING...");
  Serial.println("==============================================\n");
  
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
  
  pinMode(HALL_LEFT_PIN, INPUT_ANALOG);
  pinMode(HALL_RIGHT_PIN, INPUT_ANALOG);
  
  analogWrite(STEER_RPWM, 0);
  analogWrite(STEER_LPWM, 0);
  analogWrite(DRIVE_RPWM, 0);
  analogWrite(DRIVE_LPWM, 0);

  attachInterrupt(digitalPinToInterrupt(STEER_ENC_A), steer_updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEER_ENC_B), steer_updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_A), drive_encoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_B), drive_encoderInterrupt, CHANGE);

  steer_lastTime = micros();
  drive_lastPidTime = micros();
  drive_lastRpmTime = millis();

  Serial.println("✅ HARDWARE INITIALIZED");
  Serial.println("🔍 STARTING HOMING...\n");
  
  // JALANKAN HOMING
  steering_homing();
  
  Serial.println("\n✅ HOMING COMPLETE!");
  Serial.println("\n📝 FORMAT INPUT: A[angle]R[rpm]");
  Serial.println("   Contoh: A45R20 -> Steering 45°, Maju 20 RPM");
  Serial.println("   STOP -> Emergency stop");
  Serial.println("   HOME -> Homing ulang");
  Serial.println("==============================================\n");

  xTaskCreate(steeringTask, "Steering", 256, NULL, 2, &steeringTaskHandle);
  xTaskCreate(drivingTask, "Driving", 256, NULL, 2, &drivingTaskHandle);
  xTaskCreate(commandTask, "Command", 128, NULL, 3, &commandTaskHandle);
  xTaskCreate(displayTask, "Display", 128, NULL, 1, &displayTaskHandle);

  vTaskStartScheduler();

  while(1);
}

void loop() {}  