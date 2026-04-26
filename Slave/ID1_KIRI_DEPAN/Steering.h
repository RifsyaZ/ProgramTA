#ifndef STEERING_H
#define STEERING_H

#include "VAR_GLOBAL.h"

// Function prototypes
void steeringTask(void *pvParameters);
void steer_updateEncoder();
void steer_setMotor(int pwm);
float steer_normalizeSwerveAngle(float angle);
float steer_calculateErrorWithPath(float target, float current);
float steer_smoothError(float newError);
float steer_calculateSoftLanding(float error);
float steer_calculateFeedForward(float error);
void steer_setTargetWithSpeed(float angle, float speed);
bool steer_isValidAngle(float angle);
float steer_getCurrentError();
void steer_emergencyStop();

// ==================== STEERING IMPLEMENTATIONS ====================
#pragma region Steering_Implementations

void steer_updateEncoder() {
  int MSB = digitalRead(STEER_ENC_A);
  int LSB = digitalRead(STEER_ENC_B);
  
  int encoded = (MSB << 1) | LSB;
  int sum = (steerLastEncoded << 2) | encoded;
  
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) steerEncoderCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) steerEncoderCount--;
  
  steerLastEncoded = encoded;
  steerAngleDeg = steerEncoderCount * (360.0 / STEER_PULSE_PER_REV);
  
  while (steerAngleDeg >= 360) steerAngleDeg -= 360;
  while (steerAngleDeg < 0) steerAngleDeg += 360;
  if (steerAngleDeg > 180) steerAngleDeg = steerAngleDeg - 360;
}

float steer_getCurrentError() {
  float error = steerTargetAngle - steerAngleDeg;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  return error;
}

void steer_setMotor(int pwm) {
  pwm = constrain(pwm, -STEER_PWM_MAX, STEER_PWM_MAX);
  
  if (pwm > 0) {
    analogWrite(STEER_RPWM, abs(pwm));
    analogWrite(STEER_LPWM, 0);
  } else if (pwm < 0) {
    analogWrite(STEER_RPWM, 0);
    analogWrite(STEER_LPWM, abs(pwm));
  } else {
    analogWrite(STEER_RPWM, 0);
    analogWrite(STEER_LPWM, 0);
  }
}

bool steer_isValidAngle(float angle) {
  return (angle >= 0 && angle <= 360);
}

float steer_normalizeSwerveAngle(float angle) {
  while (angle >= 360) angle -= 360;
  while (angle < 0) angle += 360;
  
  float result;
  if (angle > 180) {
    result = angle - 360;
  } else {
    result = angle;
  }
  
  if (result > 90) {
    result = result - 180;
  } else if (result < -90) {
    result = result + 180;
  }
  
  return result;
}

float steer_calculateErrorWithPath(float target, float current) {
  float error = target - current;
  
  if (current > 85.0 && target < -85.0) {
    error = -180.0;
  } else if (current < -85.0 && target > 85.0) {
    error = 180.0;
  } else {
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
  }
  
  return error;
}

float steer_smoothError(float newError) {
  steer_errorTotal -= steer_errorHistory[steer_readIndex];
  steer_errorHistory[steer_readIndex] = newError;
  steer_errorTotal += steer_errorHistory[steer_readIndex];
  steer_readIndex = (steer_readIndex + 1) % STEER_NUM_READINGS;
  return steer_errorTotal / STEER_NUM_READINGS;
}

float steer_calculateSoftLanding(float error) {
  if (abs(error) < 15.0 && !steer_isLandingMode) {
    steer_isLandingMode = true;
    steer_landingStartTime = millis();
    steer_lastErrorForLanding = error;
  }
  
  if (steer_isLandingMode) {
    float landingTime = (millis() - steer_landingStartTime) / 1000.0;
    float landingFactor = 1.0;
    
    if (landingTime < 0.1) landingFactor = 1.0;
    else if (landingTime < 0.3) landingFactor = 0.7;
    else if (landingTime < 0.5) landingFactor = 0.5;
    else landingFactor = 0.3;
    
    if (error * steer_lastErrorForLanding < 0) {
      landingFactor *= 0.5;
    }
    
    steer_lastErrorForLanding = error;
    return landingFactor;
  }
  
  return 1.0;
}

float steer_calculateFeedForward(float error) {
  return error * 0.3;
}

void steer_setTargetWithSpeed(float angle, float speed) {
  if (!steer_isValidAngle(angle)) {
    return;
  }
  
  steerRawTargetAngle = angle;
  steerTargetAngle = constrain(steer_normalizeSwerveAngle(angle), STEER_MIN_ANGLE, STEER_MAX_ANGLE);
  steerTargetSpeed = constrain(speed, 0, 100);
  
  if (abs(steerTargetAngle - steerAngleDeg) > 45) steer_integral = 0;
  
  steer_isLandingMode = false;
  steer_targetReached = false;
  
  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.print("🎯 STEERING TARGET: ");
    Serial.print(steerRawTargetAngle, 1);
    Serial.print("° → ");
    Serial.print(steerTargetAngle, 1);
    Serial.println("°");
    xSemaphoreGive(serialMutex);
  }
}

void steer_emergencyStop() {
  steer_setMotor(0);
  steer_targetReached = false;
  steer_integral = 0;
  steering_active = false;
}

void steeringTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(2);
  SwerveCommand_t receivedCmd;
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    if (xQueueReceive(steerCommandQueue, &receivedCmd, 0) == pdTRUE) {
      if (receivedCmd.stop_all) {
        steer_setMotor(0);
        steer_integral = 0;
        steer_isLandingMode = false;
        steering_active = false;
        xQueueSend(driveCommandQueue, &receivedCmd, 0);
      } else {
        steer_setTargetWithSpeed(receivedCmd.angle, 100);
        requested_angle = receivedCmd.angle;
        requested_rpm = receivedCmd.rpm;
        command_start_time = millis();
        steering_active = true;
      }
    }
    
    if (steering_active) {
      float rawError = steer_calculateErrorWithPath(steerTargetAngle, steerAngleDeg);
      float error = steer_smoothError(rawError);
      float landingFactor = steer_calculateSoftLanding(error);
      
      float Kp, Ki, Kd;
      if (abs(rawError) <= 5.0) {
        Kp = steer_Kp_dekat;
        Ki = steer_Ki_dekat;
        Kd = steer_Kd_dekat;
      } else {
        Kp = steer_Kp_normal;
        Ki = steer_Ki_normal;
        Kd = steer_Kd_normal;
      }
      
      float pGain = Kp;
      if (steer_isLandingMode) pGain = Kp * landingFactor;
      
      float proportional = pGain * error;
      
      if (abs(error) < 10 && abs(error) > 1 && steer_isLandingMode) {
        steer_integral += error * (xFrequency * portTICK_PERIOD_MS / 1000.0) * 0.1;
        steer_integral = constrain(steer_integral, -10.0, 10.0);
      } else {
        steer_integral = 0;
      }
      
      float derivative = Kd * (error - steer_prevError) / (xFrequency * portTICK_PERIOD_MS / 1000.0);
      if (steer_isLandingMode) derivative = derivative * 0.7;
      
      float pwmPID = proportional + Ki * steer_integral + derivative;
      
      float speedMult = 1.0 + (steerTargetSpeed / 200.0);
      float baseOutput = (error > 0) ? STEER_BASE_PWM * speedMult : -STEER_BASE_PWM * speedMult;
      
      float baseWeight, pidWeight;
      if (abs(rawError) > 30) {
        baseWeight = 0.95; pidWeight = 0.05;
      } else if (abs(rawError) > 15) {
        baseWeight = 0.85; pidWeight = 0.15;
      } else if (abs(rawError) > 5) {
        baseWeight = 0.7; pidWeight = 0.3;
      } else {
        baseWeight = 0.4; pidWeight = 0.6;
      }
      
      if (steer_isLandingMode) baseWeight = baseWeight * landingFactor;
      
      float targetOutput = (baseOutput * baseWeight) + (pwmPID * pidWeight);
      targetOutput += steer_calculateFeedForward(error);
      
      if (abs(targetOutput) < STEER_MIN_PWM && abs(error) > STEER_TOLERANCE) {
        if (targetOutput > 0) targetOutput = STEER_MIN_PWM;
        else targetOutput = -STEER_MIN_PWM;
      }
      
      float maxAccel = 50 * speedMult;
      if (steer_isLandingMode) {
        if (abs(error) < 3) maxAccel = 8;
        else if (abs(error) < 8) maxAccel = 15;
      }
      
      if (targetOutput > steer_currentOutput) {
        steer_currentOutput += maxAccel;
        if (steer_currentOutput > targetOutput) steer_currentOutput = targetOutput;
      } else if (targetOutput < steer_currentOutput) {
        steer_currentOutput -= maxAccel;
        if (steer_currentOutput < targetOutput) steer_currentOutput = targetOutput;
      }
      
      steer_currentOutput = constrain(steer_currentOutput, -STEER_PWM_MAX, STEER_PWM_MAX);
      steer_setMotor((int)steer_currentOutput);
      
      steer_prevError = error;
      steer_lastError = error;
      
      if (abs(rawError) < STEER_TOLERANCE && !steer_targetReached) {
        steer_targetReached = true;
      } else if (abs(rawError) >= STEER_TOLERANCE) {
        steer_targetReached = false;
      }
    } else {
      steer_setMotor(0);
      steer_currentOutput = 0;
    }
  }
}

#pragma endregion
#endif