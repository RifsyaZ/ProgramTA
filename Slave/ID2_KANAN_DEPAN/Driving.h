#ifndef DRIVING_H
#define DRIVING_H

#include "VAR_GLOBAL.h"

// Function prototypes
void drivingTask(void *pvParameters);
void drive_encoderInterrupt();
void drive_calculateRpm();
void drive_calculatePID();
void drive_applyOutput();
void drive_setTargetRpm(float rpm, bool forward);
void drive_stop();
bool drive_determineDirection(float angle);

// ==================== DRIVING IMPLEMENTATIONS ====================
#pragma region Driving_Implementations

void drive_encoderInterrupt() {
  uint32_t now = micros();
  if (now - drive_lastInterruptTime < 100) return;
  drive_lastInterruptTime = now;
  
  int a = digitalRead(DRIVE_ENC_A);
  int b = digitalRead(DRIVE_ENC_B);
  
  static int lastA = 0;
  if (a != lastA) {
    if (drive_encoderReverse) {
      if (a == b) driveEncoderCount--;
      else driveEncoderCount++;
    } else {
      if (a == b) driveEncoderCount++;
      else driveEncoderCount--;
    }
    lastA = a;
  }
}

void drive_calculateRpm() {
  noInterrupts();
  int32_t currentPosition = driveEncoderCount;
  interrupts();
  
  int32_t deltaPosition = currentPosition - drive_lastPosition;
  
  float motorRevolutions = (float)deltaPosition / DRIVE_PULSE_PER_REV;
  drive_motorRpm = motorRevolutions * 600.0;
  
  drive_rpmFiltered = (drive_filterAlpha * drive_motorRpm) + ((1 - drive_filterAlpha) * drive_rpmFiltered);
  drive_motorRpm = drive_rpmFiltered;
  
  drive_wheelRpm = drive_motorRpm / DRIVE_GEARBOX_RATIO;
  
  drive_lastPosition = currentPosition;
}

void drive_calculatePID() {
  float error = drive_motorRpmTarget - drive_motorRpm;
  
  unsigned long now = micros();
  float dt = (now - drive_lastPidTime) / 1000000.0;
  if (dt > 0.05) dt = 0.05;
  if (dt < 0.001) dt = 0.001;
  
  drive_integral += error * dt;
  if (drive_integral > DRIVE_INTEGRAL_MAX) drive_integral = DRIVE_INTEGRAL_MAX;
  if (drive_integral < DRIVE_INTEGRAL_MIN) drive_integral = DRIVE_INTEGRAL_MIN;
  
  float derivative = (error - drive_prevError) / dt;
  if (derivative > 300) derivative = 300;
  if (derivative < -300) derivative = -300;
  
  drive_pidOutput = drive_Kp * error + drive_Ki * drive_integral + drive_Kd * derivative;
  
  drive_prevError = error;
  drive_lastPidTime = now;
}

void drive_applyOutput() {
  int pwmValue = constrain(abs(drive_pidOutput), DRIVE_PWM_MIN, DRIVE_PWM_MAX);
  
  // BALIK ARAH: MAJU pakai LPWM, MUNDUR pakai RPWM
  if (drive_wheelRpmTarget_requested > 0) {  // MAJU
    analogWrite(DRIVE_RPWM, 0);
    analogWrite(DRIVE_LPWM, pwmValue);
    drive_directionForward = true;
  } 
  else {  // MUNDUR
    analogWrite(DRIVE_RPWM, pwmValue);
    analogWrite(DRIVE_LPWM, 0);
    drive_directionForward = false;
  }
  
  drive_pwmOutput = pwmValue;
}

bool drive_determineDirection(float angle) {
  while (angle >= 360) angle -= 360;
  while (angle < 0) angle += 360;
  
  if ((angle >= 0 && angle <= 90) || (angle >= 270 && angle <= 360)) {
    return true;  // MAJU
  } else {
    return false; // MUNDUR
  }
}

void drive_setTargetRpm(float rpm, bool forward) {
  drive_wheelRpmTarget_requested = forward ? rpm : -rpm;
  drive_directionForward = forward;
  
  if (rpm == 0) {
    drive_stop();
    return;
  }
  
  float rpmAbs = abs(rpm);
  if (rpmAbs > DRIVE_MAX_WHEEL_RPM) {
    rpmAbs = DRIVE_MAX_WHEEL_RPM;
  }
  
  if (forward) {
    drive_wheelRpmTarget = rpmAbs;
  } else {
    drive_wheelRpmTarget = -rpmAbs;
  }
  
  drive_motorRpmTarget = drive_wheelRpmTarget * DRIVE_GEARBOX_RATIO;
  
  drive_motorRunning = true;
  drive_prevError = 0;
  drive_integral = 0;
  
  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.print("⚡ DRIVING TARGET: ");
    Serial.print(rpmAbs, 1);
    Serial.print(" RPM ");
    Serial.print(forward ? "MAJU" : "MUNDUR");
    Serial.print(" (sudut ");
    Serial.print(steerRawTargetAngle, 1);
    Serial.println("°)");
    xSemaphoreGive(serialMutex);
  }
}

void drive_stop() {
  drive_motorRunning = false;
  driving_active = false;
  analogWrite(DRIVE_RPWM, 0);
  analogWrite(DRIVE_LPWM, 0);
  drive_pwmOutput = 0;
  drive_pidOutput = 0;
  drive_motorRpmTarget = 0;
  drive_wheelRpmTarget = 0;
  drive_wheelRpmTarget_requested = 0;
  drive_integral = 0;
  
  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("⏹️ DRIVING STOP");
    xSemaphoreGive(serialMutex);
  }
}

void drivingTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);
  SwerveCommand_t receivedCmd;
  float local_target_rpm = 0;
  bool local_forward = true;
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    if (xQueueReceive(driveCommandQueue, &receivedCmd, 0) == pdTRUE) {
      if (receivedCmd.stop_all) {
        drive_stop();
        local_target_rpm = 0;
        driving_active = false;
      } else {
        local_forward = drive_determineDirection(receivedCmd.angle);
        local_target_rpm = receivedCmd.rpm;
        
        if (local_target_rpm != 0) {
          drive_setTargetRpm(local_target_rpm, local_forward);
          driving_active = true;
        } else {
          drive_stop();
          driving_active = false;
        }
      }
    }
    
    drive_calculateRpm();
    
    if (driving_active) {
      drive_calculatePID();
      drive_applyOutput();
    } else {
      analogWrite(DRIVE_RPWM, 0);
      analogWrite(DRIVE_LPWM, 0);
    }
    
    // Kirim encoder count driving realtime ke Serial2
    Serial2.print("ENC:");
    Serial2.println(driveEncoderCount);
  }
}

#pragma endregion
#endif