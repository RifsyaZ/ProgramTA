#ifndef HOMING_H
#define HOMING_H

#include "VAR_GLOBAL.h"

// ==================== HOMING PIN DEFINITIONS ====================
#define HALL_LEFT_PIN   PA0
#define HALL_RIGHT_PIN  PA1

// ==================== HOMING CONSTANTS ====================
#define HOMING_TIMEOUT_MS   5000
#define HOMING_PWM           30
#define HOMING_SAMPLE_DELAY  5

#define HALL_HIGH_THRESHOLD  1000
#define HALL_LOW_THRESHOLD   450
#define HALL_GAP_MIN         600
#define HALL_GAP_MAX         800

enum HomingState {
  HOMING_IDLE,
  HOMING_START,
  HOMING_MOVING_CCW,
  HOMING_MOVING_CW,
  HOMING_COMPLETE,
  HOMING_FAILED
};

extern HomingState homingState;
extern bool homingComplete;
extern unsigned long homingStartTime;
extern int lastLeftValue;
extern int lastRightValue;

// Function prototypes
void steering_homing();
void steering_homing_task(void *pvParameters);
void readHallSensors(int &leftVal, int &rightVal);
bool checkHomeCondition(int leftVal, int rightVal);
bool checkLeftSideCondition(int leftVal, int rightVal);
bool checkRightSideCondition(int leftVal, int rightVal);
bool checkGapCondition(int leftVal, int rightVal);
void stopSteeringMotor();
void moveSteeringCCW(int pwm);
void moveSteeringCW(int pwm);
void resetSteeringEncoder();

// ==================== IMPLEMENTATIONS ====================
#pragma region Homing_Implementations

void readHallSensors(int &leftVal, int &rightVal) {
  leftVal = analogRead(HALL_LEFT_PIN);
  rightVal = analogRead(HALL_RIGHT_PIN);
}

bool checkHomeCondition(int leftVal, int rightVal) {
  return (leftVal > HALL_HIGH_THRESHOLD && rightVal < HALL_LOW_THRESHOLD);
}

bool checkLeftSideCondition(int leftVal, int rightVal) {
  return (leftVal > HALL_HIGH_THRESHOLD && rightVal > HALL_HIGH_THRESHOLD);
}

bool checkRightSideCondition(int leftVal, int rightVal) {
  return (leftVal < HALL_LOW_THRESHOLD && rightVal < HALL_LOW_THRESHOLD);
}

bool checkGapCondition(int leftVal, int rightVal) {
  return (leftVal >= HALL_GAP_MIN && leftVal <= HALL_GAP_MAX &&
          rightVal >= HALL_GAP_MIN && rightVal <= HALL_GAP_MAX);
}

void stopSteeringMotor() {
  analogWrite(STEER_RPWM, 0);
  analogWrite(STEER_LPWM, 0);
  steer_currentOutput = 0;
}

void moveSteeringCCW(int pwm) {
  analogWrite(STEER_RPWM, 0);
  analogWrite(STEER_LPWM, pwm);
  steer_currentOutput = -pwm;
}

void moveSteeringCW(int pwm) {
  analogWrite(STEER_RPWM, pwm);
  analogWrite(STEER_LPWM, 0);
  steer_currentOutput = pwm;
}

void resetSteeringEncoder() {
  steerEncoderCount = 0;
  steerAngleDeg = 0;
  steerTargetAngle = 0;
  steerRawTargetAngle = 0;
  steer_targetReached = true;
  steer_integral = 0;
  steer_prevError = 0;
  steer_lastError = 0;
  steer_isLandingMode = false;
}

void steering_homing() {
  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("\n╔════════════════════════════════════════════╗");
    Serial.println("║        STEERING HOMING STARTED             ║");
    Serial.println("╚════════════════════════════════════════════╝");
    Serial.println("🔍 Mencari posisi HOME (0°)...");
    xSemaphoreGive(serialMutex);
  }
  
  homingState = HOMING_START;
  homingComplete = false;
  homingStartTime = millis();
  
  int leftVal, rightVal;
  bool homeDetected = false;
  
  steering_active = false;
  
  while (!homeDetected && (millis() - homingStartTime) < HOMING_TIMEOUT_MS) {
    readHallSensors(leftVal, rightVal);
    
    // Debug setiap 200ms
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 200 && serialMutex != NULL) {
      lastDebug = millis();
      xSemaphoreTake(serialMutex, portMAX_DELAY);
      Serial.print("📊 L:");
      Serial.print(leftVal);
      Serial.print(" R:");
      Serial.print(rightVal);
      Serial.print(" | ");
      switch(homingState) {
        case HOMING_START: Serial.print("START"); break;
        case HOMING_MOVING_CCW: Serial.print("← CCW"); break;
        case HOMING_MOVING_CW: Serial.print("CW →"); break;
        default: Serial.print("?"); break;
      }
      Serial.println();
      xSemaphoreGive(serialMutex);
    }
    
    switch(homingState) {
      case HOMING_START:
        if (checkHomeCondition(leftVal, rightVal)) {
          stopSteeringMotor();
          resetSteeringEncoder();
          homeDetected = true;
          homingState = HOMING_COMPLETE;
          if (serialMutex != NULL) {
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.println("✅ HOME DETECTED!");
            xSemaphoreGive(serialMutex);
          }
        }
        else if (checkLeftSideCondition(leftVal, rightVal)) {
          homingState = HOMING_MOVING_CCW;
          if (serialMutex != NULL) {
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.println("📍 SISI KIRI → CCW ←");
            xSemaphoreGive(serialMutex);
          }
        }
        else if (checkRightSideCondition(leftVal, rightVal)) {
          homingState = HOMING_MOVING_CW;
          if (serialMutex != NULL) {
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.println("📍 SISI KANAN → CW →");
            xSemaphoreGive(serialMutex);
          }
        }
        else {
          homingState = HOMING_MOVING_CCW;
        }
        break;
        
      case HOMING_MOVING_CCW:
        moveSteeringCCW(HOMING_PWM);
        if (checkHomeCondition(leftVal, rightVal)) {
          stopSteeringMotor();
          resetSteeringEncoder();
          homeDetected = true;
          homingState = HOMING_COMPLETE;
          if (serialMutex != NULL) {
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.println("✅ HOME DETECTED! (CCW)");
            xSemaphoreGive(serialMutex);
          }
        }
        else if (checkRightSideCondition(leftVal, rightVal)) {
          homingState = HOMING_MOVING_CW;
        }
        break;
        
      case HOMING_MOVING_CW:
        moveSteeringCW(HOMING_PWM);
        if (checkHomeCondition(leftVal, rightVal)) {
          stopSteeringMotor();
          resetSteeringEncoder();
          homeDetected = true;
          homingState = HOMING_COMPLETE;
          if (serialMutex != NULL) {
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.println("✅ HOME DETECTED! (CW)");
            xSemaphoreGive(serialMutex);
          }
        }
        else if (checkLeftSideCondition(leftVal, rightVal)) {
          homingState = HOMING_MOVING_CCW;
        }
        break;
        
      default:
        break;
    }
    
    vTaskDelay(pdMS_TO_TICKS(HOMING_SAMPLE_DELAY));
  }
  
  if (!homeDetected) {
    homingState = HOMING_FAILED;
    stopSteeringMotor();
    if (serialMutex != NULL) {
      xSemaphoreTake(serialMutex, portMAX_DELAY);
      Serial.println("❌ HOMING FAILED! Check sensors!");
      xSemaphoreGive(serialMutex);
    }
  }
  
  homingComplete = true;
  
  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("╔════════════════════════════════════════════╗");
    Serial.println("║        HOMING COMPLETED!                  ║");
    Serial.println("╚════════════════════════════════════════════╝");
    Serial.print("📍 POSISI: ");
    Serial.print(steerAngleDeg, 1);
    Serial.println("°");
    xSemaphoreGive(serialMutex);
  }
  
  steering_active = true;
}

void steering_homing_task(void *pvParameters) {
  steering_homing();
  vTaskDelete(NULL);
}

#pragma endregion
#endif