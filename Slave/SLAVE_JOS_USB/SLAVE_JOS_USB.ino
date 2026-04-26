/**
 * SWERVE DRIVE CONTROL SYSTEM - PARALLEL EXECUTION
 * Steering dan Driving bekerja BERSAMAAN
 * Tidak menunggu steering mencapai target
 * 
 * Input: A[angle]R[rpm]  Contoh: A90R50
 * 
 * LOGIKA ARAH:
 * - Sudut 0-90° dan 270-360° → MAJU (RPM positif)
 * - Sudut 90-270° → MUNDUR (RPM negatif)
 */

#include <Arduino.h>
#include <STM32FreeRTOS.h>

// ==================== PIN DEFINITIONS ====================
// Steering motor (Position control) - BTS7960
#define STEER_ENC_A PB2
#define STEER_ENC_B PB1
#define STEER_ENA PB14
#define STEER_IN1 PB13
#define STEER_IN2 PB12

// Driving motor - BTS7960
#define DRIVE_ENC_A PB6
#define DRIVE_ENC_B PB7
#define DRIVE_RPWM PB10
#define DRIVE_LPWM PA7

// LED Indikator
#define LED_PIN PC13

// ==================== STEERING CONSTANTS ====================
#define STEER_PULSE_PER_REV 1813.0
#define STEER_PWM_MAX 255
#define STEER_BASE_PWM 120
#define STEER_MIN_PWM 50

// Steering angle limits
const float STEER_MIN_ANGLE = -90.0;
const float STEER_MAX_ANGLE = 90.0;
const float STEER_TOLERANCE = 0.5;           // Toleransi final

// Steering PID Parameters
float steer_Kp_normal = 3.0;
float steer_Ki_normal = 0.0;
float steer_Kd_normal = 2.0;

float steer_Kp_dekat = 5.5;
float steer_Ki_dekat = 0.5;
float steer_Kd_dekat = 2.0;

// ==================== DRIVING CONSTANTS ====================
const float DRIVE_PULSE_PER_REV = 14.0;
const float DRIVE_GEARBOX_RATIO = 6.77;
const float DRIVE_WHEEL_DIAMETER_MM = 82.0;
const float DRIVE_MAX_WHEEL_RPM = 800.0;

// Driving PID Parameters
float drive_Kp = 0.05;
float drive_Ki = 0.5;
float drive_Kd = 0.001;

// ==================== GLOBAL VARIABLES ====================
// Steering variables
volatile int32_t steerEncoderCount = 0;
int steerLastEncoded = 0;
float steerAngleDeg = 0;
float steerTargetAngle = 0;
float steerRawTargetAngle = 0;  // Angle dalam 0-360°
float steerTargetSpeed = 100;

// Steering PID variables
float steer_lastError = 0;
float steer_integral = 0;
float steer_prevError = 0;
float steer_currentOutput = 0;
unsigned long steer_lastTime = 0;
bool steer_targetReached = false;

// Steering soft landing
bool steer_isLandingMode = false;
unsigned long steer_landingStartTime = 0;
float steer_lastErrorForLanding = 0;

// Driving variables
volatile int32_t driveEncoderCount = 0;
volatile uint32_t drive_lastInterruptTime = 0;
int32_t drive_lastPosition = 0;
uint32_t drive_lastRpmTime = 0;

// Driving RPM variables
float drive_motorRpm = 0.0;
float drive_wheelRpm = 0.0;
float drive_wheelRpmTarget = 0.0;
float drive_wheelRpmTarget_requested = 0.0;
float drive_motorRpmTarget = 0.0;
float drive_rpmFiltered = 0;
const float drive_filterAlpha = 0.2;

// Driving direction based on angle
bool drive_directionForward = true;  // true = maju, false = mundur
float drive_requestedRpm = 0;        // RPM yang diminta user (selalu positif)

// Driving PID variables
float drive_integral = 0;
float drive_prevError = 0;
uint32_t drive_lastPidTime = 0;
float drive_pidOutput = 0;
int drive_pwmOutput = 0;
bool drive_motorRunning = false;
bool drive_direction = true;
bool drive_encoderReverse = true;

// ============== VARIABEL STATUS ==============
bool steering_active = false;
bool driving_active = false;
unsigned long command_start_time = 0;
float requested_angle = 0;
float requested_rpm = 0;

// Queue untuk komunikasi antar task
QueueHandle_t steerCommandQueue;
QueueHandle_t driveCommandQueue;

// ============== STRUCT UNTUK COMMAND ==============
typedef struct {
  float angle;      // Angle dalam 0-360°
  float rpm;        // RPM yang diminta (selalu positif)
  bool stop_all;
} SwerveCommand_t;

// Moving average untuk steering
const int STEER_NUM_READINGS = 3;
float steer_errorHistory[STEER_NUM_READINGS];
int steer_readIndex = 0;
float steer_errorTotal = 0;

// Driving limits
const float DRIVE_INTEGRAL_MAX = 500.0;
const float DRIVE_INTEGRAL_MIN = -500.0;
const int DRIVE_PWM_MIN = 15;
const int DRIVE_PWM_MAX = 255;

// Command parsing
char inputBuffer[50];
int bufferIndex = 0;

// ==================== TASK HANDLES ====================
TaskHandle_t steeringTaskHandle;
TaskHandle_t drivingTaskHandle;
TaskHandle_t commandTaskHandle;
TaskHandle_t displayTaskHandle;

// ==================== MUTEX ====================
SemaphoreHandle_t serialMutex;

// ==================== FUNCTION PROTOTYPES ====================
void steeringTask(void *pvParameters);
void drivingTask(void *pvParameters);
void commandTask(void *pvParameters);
void displayTask(void *pvParameters);

// Steering functions
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

// Driving functions
void drive_encoderInterrupt();
void drive_calculateRpm();
void drive_calculatePID();
void drive_applyOutput();
void drive_setTargetRpm(float rpm, bool forward);
void drive_stop();

// Direction determination
bool drive_determineDirection(float angle);  // Menentukan arah berdasarkan sudut

// Common functions
void parseCommand(String cmd);
void printHelp();
void printSystemStatus();
void executeSwerveCommand(float angle, float rpm);

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Buat queue untuk komunikasi antar task
  steerCommandQueue = xQueueCreate(5, sizeof(SwerveCommand_t));
  driveCommandQueue = xQueueCreate(5, sizeof(SwerveCommand_t));
  
  // Inisialisasi mutex
  serialMutex = xSemaphoreCreateMutex();
  
  // Inisialisasi pin
  pinMode(STEER_ENC_A, INPUT_PULLUP);
  pinMode(STEER_ENC_B, INPUT_PULLUP);
  pinMode(STEER_ENA, OUTPUT);
  pinMode(STEER_IN1, OUTPUT);
  pinMode(STEER_IN2, OUTPUT);
  
  pinMode(DRIVE_ENC_A, INPUT_PULLUP);
  pinMode(DRIVE_ENC_B, INPUT_PULLUP);
  pinMode(DRIVE_RPWM, OUTPUT);
  pinMode(DRIVE_LPWM, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Inisialisasi interrupt
  attachInterrupt(digitalPinToInterrupt(STEER_ENC_A), steer_updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEER_ENC_B), steer_updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_A), drive_encoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_B), drive_encoderInterrupt, CHANGE);
  
  // Inisialisasi history
  for (int i = 0; i < STEER_NUM_READINGS; i++) {
    steer_errorHistory[i] = 0;
  }
  
  steerEncoderCount = 0;
  steerAngleDeg = 0;
  steer_lastTime = micros();
  
  drive_lastPidTime = micros();
  drive_lastRpmTime = millis();
  
  // Tampilkan informasi sistem
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║     SWERVE DRIVE CONTROL SYSTEM v7.0      ║");
  Serial.println("║          PARALLEL EXECUTION                ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\n✅ STEERING: Rate 2ms");
  Serial.println("✅ DRIVING: Rate 20ms");
  Serial.println("✅ Kedua motor bekerja BERSAMAAN");
  
  Serial.println("\n📌 LOGIKA ARAH DRIVING:");
  Serial.println("   • Sudut 0-90° & 270-360° → MAJU (RPM positif)");
  Serial.println("   • Sudut 90-270° → MUNDUR (RPM negatif)");
  
  Serial.println("\n📝 FORMAT INPUT: A[angle]R[rpm]");
  Serial.println("   Contoh: A45R20  -> Steering 45°, Maju 20 RPM");
  Serial.println("           A225R20 -> Steering 225°, Mundur 20 RPM");
  Serial.println("==============================================\n");
  xSemaphoreGive(serialMutex);
  
  // Buat task FreeRTOS
  xTaskCreate(
    steeringTask,
    "Steering Control",
    256,
    NULL,
    2,                    // Prioritas SEDANG (2)
    &steeringTaskHandle
  );
  
  xTaskCreate(
    drivingTask,
    "Driving Control",
    256,
    NULL,
    2,                    // Prioritas SEDANG (2) - SAMA dengan steering
    &drivingTaskHandle
  );
  
  xTaskCreate(
    commandTask,
    "Command Parser",
    128,
    NULL,
    3,                    // Prioritas TERTINGGI (3)
    &commandTaskHandle
  );
  
  xTaskCreate(
    displayTask,
    "Display",
    128,
    NULL,
    1,                    // Prioritas RENDAH (1)
    &displayTaskHandle
  );
  
  // Mulai scheduler
  vTaskStartScheduler();
  
  while(1);
}

// ==================== TASK: STEERING CONTROL ====================
void steeringTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(2);  // 2ms interval
  
  SwerveCommand_t receivedCmd;
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // Cek apakah ada command baru dari queue
    if (xQueueReceive(steerCommandQueue, &receivedCmd, 0) == pdTRUE) {
      if (receivedCmd.stop_all) {
        // Emergency stop
        steer_setMotor(0);
        steer_integral = 0;
        steer_isLandingMode = false;
        steering_active = false;
        
        // Kirim perintah stop ke driving
        xQueueSend(driveCommandQueue, &receivedCmd, 0);
      } else {
        // Set target baru
        steer_setTargetWithSpeed(receivedCmd.angle, 100);
        requested_angle = receivedCmd.angle;
        requested_rpm = receivedCmd.rpm;
        command_start_time = millis();
        steering_active = true;
      }
    }
    
    // Jika steering aktif, lakukan kontrol
    if (steering_active) {
      // Hitung error steering
      float rawError = steer_calculateErrorWithPath(steerTargetAngle, steerAngleDeg);
      float error = steer_smoothError(rawError);
      
      // Hitung landing factor
      float landingFactor = steer_calculateSoftLanding(error);
      
      // Pilih PID berdasarkan error
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
      
      // PID Steering calculation
      float pGain = Kp;
      if (steer_isLandingMode) pGain = Kp * landingFactor;
      
      float proportional = pGain * error;
      
      // Integral dengan anti-windup
      if (abs(error) < 10 && abs(error) > 1 && steer_isLandingMode) {
        steer_integral += error * (xFrequency * portTICK_PERIOD_MS / 1000.0) * 0.1;
        steer_integral = constrain(steer_integral, -10.0, 10.0);
      } else {
        steer_integral = 0;
      }
      
      float derivative = Kd * (error - steer_prevError) / (xFrequency * portTICK_PERIOD_MS / 1000.0);
      if (steer_isLandingMode) derivative = derivative * 0.7;
      
      float pwmPID = proportional + Ki * steer_integral + derivative;
      
      // Base PWM
      float speedMult = 1.0 + (steerTargetSpeed / 200.0);
      float baseOutput = (error > 0) ? STEER_BASE_PWM * speedMult : -STEER_BASE_PWM * speedMult;
      
      // Weight dinamis
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
      
      // Minimum PWM
      if (abs(targetOutput) < STEER_MIN_PWM && abs(error) > STEER_TOLERANCE) {
        if (targetOutput > 0) targetOutput = STEER_MIN_PWM;
        else targetOutput = -STEER_MIN_PWM;
      }
      
      // Soft start/stop
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
      
      // Cek apakah sudah mencapai target
      if (abs(rawError) < STEER_TOLERANCE && !steer_targetReached) {
        steer_targetReached = true;
        if (serialMutex != NULL && xSemaphoreTake(serialMutex, 0) == pdTRUE) {
          Serial.println("🟢 STEERING ON TARGET");
          xSemaphoreGive(serialMutex);
        }
      } else if (abs(rawError) >= STEER_TOLERANCE) {
        steer_targetReached = false;
      }
    } else {
      // Jika tidak aktif, matikan motor
      steer_setMotor(0);
      steer_currentOutput = 0;
    }
  }
}

// ==================== TASK: DRIVING CONTROL ====================
void drivingTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);  // 20ms interval
  
  SwerveCommand_t receivedCmd;
  float local_target_rpm = 0;
  bool local_forward = true;
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // Cek command dari queue
    if (xQueueReceive(driveCommandQueue, &receivedCmd, 0) == pdTRUE) {
      if (receivedCmd.stop_all) {
        drive_stop();
        local_target_rpm = 0;
        driving_active = false;
      } else {
        // Tentukan arah berdasarkan sudut
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
    
    // Hitung RPM
    drive_calculateRpm();
    
    // PID control jika driving aktif
    if (driving_active) {
      drive_calculatePID();
      drive_applyOutput();
    } else {
      // Pastikan motor mati jika tidak aktif
      analogWrite(DRIVE_RPWM, 0);
      analogWrite(DRIVE_LPWM, 0);
    }
  }
}

// ==================== TASK: COMMAND PARSING ====================
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

// ==================== TASK: DISPLAY ====================
void displayTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(500);
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    if (serialMutex != NULL && xSemaphoreTake(serialMutex, 0) == pdTRUE) {
      Serial.println("----------------------------------------");
      
      // STEERING INFO
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
      
      // DRIVING INFO
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
      
      if (drive_directionForward) {
        Serial.print("+");
      } else {
        Serial.print("-");
      }
      Serial.print(drive_wheelRpmTarget_requested, 1);
      
      if (driving_active) {
        float driveError = (drive_directionForward ? drive_wheelRpmTarget : -drive_wheelRpmTarget) - drive_wheelRpm;
        Serial.print(" | Err: ");
        Serial.print(driveError, 1);
        Serial.print(" | PWM: ");
        Serial.print(drive_pwmOutput);
      }
      Serial.println();
      
      // LED indikator
      if (steering_active || driving_active) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      } else {
        digitalWrite(LED_PIN, HIGH);
      }
      
      xSemaphoreGive(serialMutex);
    }
  }
}

// ==================== STEERING FUNCTIONS ====================
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
    digitalWrite(STEER_IN1, HIGH);
    digitalWrite(STEER_IN2, LOW);
  } else if (pwm < 0) {
    digitalWrite(STEER_IN1, LOW);
    digitalWrite(STEER_IN2, HIGH);
  } else {
    digitalWrite(STEER_IN1, LOW);
    digitalWrite(STEER_IN2, LOW);
  }
  
  analogWrite(STEER_ENA, abs(pwm));
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

// ==================== DRIVING FUNCTIONS ====================
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
  if (drive_pidOutput >= 0) {
    drive_direction = true;
    drive_pwmOutput = constrain(drive_pidOutput, 0, DRIVE_PWM_MAX);
    
    if (drive_pwmOutput > 0 && drive_pwmOutput < DRIVE_PWM_MIN) {
      drive_pwmOutput = DRIVE_PWM_MIN;
    }
    
    analogWrite(DRIVE_RPWM, drive_pwmOutput);
    analogWrite(DRIVE_LPWM, 0);
  } else {
    drive_direction = false;
    drive_pwmOutput = constrain(-drive_pidOutput, 0, DRIVE_PWM_MAX);
    
    if (drive_pwmOutput > 0 && drive_pwmOutput < DRIVE_PWM_MIN) {
      drive_pwmOutput = DRIVE_PWM_MIN;
    }
    
    analogWrite(DRIVE_RPWM, 0);
    analogWrite(DRIVE_LPWM, drive_pwmOutput);
  }
}

/**
 * Menentukan arah driving berdasarkan sudut (0-360°)
 * return: true = MAJU, false = MUNDUR
 */
bool drive_determineDirection(float angle) {
  // Normalisasi sudut ke 0-360°
  while (angle >= 360) angle -= 360;
  while (angle < 0) angle += 360;
  
  // Logika:
  // Maju: 0° - 90° dan 270° - 360°
  // Mundur: 90° - 270°
  if ((angle >= 0 && angle <= 90) || (angle >= 270 && angle <= 360)) {
    return true;  // MAJU
  } else {
    return false; // MUNDUR
  }
}

void drive_setTargetRpm(float rpm, bool forward) {
  drive_wheelRpmTarget_requested = rpm;
  drive_directionForward = forward;
  
  if (rpm == 0) {
    drive_stop();
    return;
  }
  
  float rpmAbs = abs(rpm);
  if (rpmAbs > DRIVE_MAX_WHEEL_RPM) {
    rpmAbs = DRIVE_MAX_WHEEL_RPM;
  }
  
  // Set target dengan arah yang sesuai
  if (forward) {
    drive_wheelRpmTarget = rpmAbs;      // Positif untuk maju
  } else {
    drive_wheelRpmTarget = -rpmAbs;     // Negatif untuk mundur
  }
  
  drive_motorRpmTarget = drive_wheelRpmTarget * DRIVE_GEARBOX_RATIO;
  
  drive_motorRunning = true;
  drive_prevError = 0;
  drive_integral = 0;  // Reset integral saat set target baru
  
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

// ==================== COMMAND EXECUTION ====================
void executeSwerveCommand(float angle, float rpm) {
  SwerveCommand_t cmd;
  cmd.angle = angle;
  cmd.rpm = rpm;
  cmd.stop_all = false;
  
  // Kirim command ke steering task
  xQueueSend(steerCommandQueue, &cmd, 0);
  
  // Kirim juga ke driving task
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
  
  // Parse format A[angle]R[rpm]
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
    Serial.println("  • Cocok untuk swerve drive 4WD");
    
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
    Serial.print(drive_wheelRpmTarget_requested, 1);
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

void loop() {
  // Empty
}