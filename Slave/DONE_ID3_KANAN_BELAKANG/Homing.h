#ifndef HOMING_H
#define HOMING_H

#include "VAR_GLOBAL.h"
#include <Arduino.h>
#include <STM32FreeRTOS.h>

// ==================== PIN DEFINITIONS ====================
#define HOMING_SENSOR_KIRI PA5   //PA4
#define HOMING_SENSOR_KANAN PA4  //PA5

// ==================== CONSTANTS ====================
#define HOMING_PID_KP 1.5
#define HOMING_PID_KI 0.5
#define HOMING_PID_KD 0.2
#define HOMING_PWM_MAX 100
#define HOMING_PWM_MIN 15
#define HOMING_RPM 20
#define HOMING_SENSOR_HIGH 1020
#define HOMING_SENSOR_LOW 300

// ==================== ARAH KONSTANTA ====================
#define ARAH_HOME 1      // Kiri HIGH, Kanan LOW -> SUDAH HOME
#define ARAH_KE_KIRI 2   // Kedua HIGH -> Magnet di KANAN -> Putar KIRI
#define ARAH_KE_KANAN 3  // Kedua LOW  -> Magnet di KIRI  -> Putar KANAN

// ==================== GLOBAL VARIABLES ====================
extern volatile bool homingActive;
extern volatile bool homingCompleted;
extern volatile int homingDirection;
extern TaskHandle_t homingTaskHandle;

// ==================== FUNCTION DECLARATIONS ====================
void homingTask(void *pvParameters);
int homing_checkPosition();
void homing_moveToHome();
void homing_stop();
void homing_start();
void homing_resetEncoder();
bool homing_isAtHome();
void homing_handleCommand(String cmd);

// ==================== IMPLEMENTATION ====================
#ifdef _VAR_GLOBAL_IMPLEMENTATION_

volatile bool homingActive = false;
volatile bool homingCompleted = false;
volatile int homingDirection = 0;
TaskHandle_t homingTaskHandle = NULL;

static float pidError = 0;
static float pidPrevError = 0;
static float pidIntegral = 0;
static float pidOutput = 0;
static unsigned long lastPidTime = 0;
static volatile int32_t lastEncoderCount = 0;
static volatile unsigned long lastRpmTime = 0;
static volatile float currentRpm = 0;

#endif  // _VAR_GLOBAL_IMPLEMENTATION_

/*
 * ============================================================
 * LOGIKA HOMING (SESUAI KODE ASLI ANDA)
 * ============================================================
 * 
 * SENSOR KIRI   SENSOR KANAN   ARTI                    AKSI
 * -----------   ------------   --------------------    --------------
 * >= 950        <= 450         MAGNET DI TENGAH        HOME -> STOP
 * >= 950        >= 950         MAGNET DI SEBELAH KANAN PUTAR KIRI
 * <= 450        <= 450         MAGNET DI SEBELAH KIRI  PUTAR KANAN
 * 
 * Jika tidak memenuhi ketiga kondisi di atas, 
 * tetap lanjut dengan arah terakhir yang dipilih.
 * ============================================================
 */

int homing_checkPosition() {
  int kiri = analogRead(HOMING_SENSOR_KIRI);
  int kanan = analogRead(HOMING_SENSOR_KANAN);

  // Kondisi 1: Kiri HIGH, Kanan LOW = HOME
  if (kanan >= HOMING_SENSOR_HIGH && kiri <= HOMING_SENSOR_LOW) {
    return ARAH_HOME;
  }

  // Kondisi 2: Kedua HIGH = Magnet di sebelah KIRI -> Putar KANAN
  if (kiri >= HOMING_SENSOR_HIGH && kanan >= HOMING_SENSOR_HIGH) {
    return ARAH_KE_KIRI;
  }

  // Kondisi 3: Kedua LOW = Magnet di sebelah KANAN -> Putar KIRI
  if (kiri <= HOMING_SENSOR_LOW && kanan <= HOMING_SENSOR_LOW) {
    return ARAH_KE_KANAN;
  }

  // Jika tidak memenuhi ketiganya, return 0 (tidak mengubah arah)
  return 0;
}

bool homing_isAtHome() {
  return (homing_checkPosition() == ARAH_HOME);
}

void homing_resetEncoder() {
  noInterrupts();
  steerEncoderCount = 0;
  interrupts();
  steerAngleDeg = 0;

  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("🔄 ENCODER RESET -> Posisi 0° (HOME)");
    xSemaphoreGive(serialMutex);
  }
}

static void homing_calculateRpm() {
  unsigned long now = millis();
  if (now - lastRpmTime < 50) return;

  int32_t currentCount;
  noInterrupts();
  currentCount = steerEncoderCount;
  interrupts();

  int32_t delta = currentCount - lastEncoderCount;
  lastEncoderCount = currentCount;

  float dt = (now - lastRpmTime) / 1000.0;
  lastRpmTime = now;

  if (dt > 0) {
    float rpm = (delta / STEER_PULSE_PER_REV) * 60.0 / dt;
    currentRpm = currentRpm * 0.7 + rpm * 0.3;
  }
}

static void homing_setMotor(int arahPutar) {
  // arahPutar: 1 = KANAN, -1 = KIRI, 0 = STOP
  if (arahPutar == 0) {
    analogWrite(STEER_RPWM, 0);
    analogWrite(STEER_LPWM, 0);
    pidIntegral = 0;
    pidOutput = 0;
    return;
  }

  unsigned long now = micros();
  float dt = (now - lastPidTime) / 1000000.0;
  if (dt > 0.05) dt = 0.05;
  if (dt < 0.001) dt = 0.001;

  pidError = HOMING_RPM - abs(currentRpm);

  if (abs(pidError) < 30) {
    pidIntegral += pidError * dt;
    pidIntegral = constrain(pidIntegral, -100, 100);
  } else {
    pidIntegral = 0;
  }

  float derivative = (pidError - pidPrevError) / dt;
  pidOutput = (HOMING_PID_KP * pidError) + (HOMING_PID_KI * pidIntegral) + (HOMING_PID_KD * derivative);

  pidPrevError = pidError;
  lastPidTime = now;

  int pwm = constrain(abs(pidOutput), HOMING_PWM_MIN, HOMING_PWM_MAX);

  if (arahPutar == 1) {
    // Putar KANAN
    analogWrite(STEER_RPWM, 0);
    analogWrite(STEER_LPWM, pwm);
  } else {
    // Putar KIRI (arahPutar == -1)
    analogWrite(STEER_RPWM, pwm);
    analogWrite(STEER_LPWM, 0);
  }
}

void homing_stop() {
  homing_setMotor(0);
}

// Variabel static untuk menyimpan arah terakhir
static int lastDirection = -1;  // Default: putar KIRI saat start

void homing_moveToHome() {
  int posisi = homing_checkPosition();

  // Jika posisi valid (1,2,3), update arah dan simpan
  if (posisi == ARAH_HOME) {
    homingDirection = ARAH_HOME;
    lastDirection = 0;  // STOP

    homing_stop();
    homingActive = false;
    homingCompleted = true;
    homing_resetEncoder();

    if (serialMutex != NULL) {
      xSemaphoreTake(serialMutex, portMAX_DELAY);
      Serial.println("\n✅ HOMING SELESAI! Encoder di-reset ke 0°\n");
      xSemaphoreGive(serialMutex);
    }
    return;
  } else if (posisi == ARAH_KE_KIRI) {
    homingDirection = ARAH_KE_KIRI;
    lastDirection = -1;  // Putar KIRI
    homing_setMotor(-1);
  } else if (posisi == ARAH_KE_KANAN) {
    homingDirection = ARAH_KE_KANAN;
    lastDirection = 1;  // Putar KANAN
    homing_setMotor(1);
  } else {
    // posisi == 0 (tidak memenuhi 3 kondisi) -> lanjutkan arah terakhir
    homing_setMotor(lastDirection);
  }
}

void homing_start() {
  if (homingActive) {
    if (serialMutex != NULL) {
      xSemaphoreTake(serialMutex, portMAX_DELAY);
      Serial.println("⚠️ Homing sudah berjalan!");
      xSemaphoreGive(serialMutex);
    }
    return;
  }

  if (homing_isAtHome()) {
    homing_resetEncoder();
    homingCompleted = true;
    if (serialMutex != NULL) {
      xSemaphoreTake(serialMutex, portMAX_DELAY);
      Serial.println("\n📍 Sudah di posisi HOME. Encoder di-reset.\n");
      xSemaphoreGive(serialMutex);
    }
    return;
  }

  homingActive = true;
  homingCompleted = false;

  // Reset PID
  pidError = 0;
  pidPrevError = 0;
  pidIntegral = 0;
  pidOutput = 0;
  currentRpm = 0;
  lastRpmTime = millis();
  lastPidTime = micros();
  lastEncoderCount = steerEncoderCount;

  // Default arah: putar KIRI dulu
  lastDirection = -1;

  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("\n╔════════════════════════════════════╗");
    Serial.println("║       🏠 MULAI HOMING              ║");
    Serial.println("║   Mencari posisi HOME...           ║");
    Serial.println("╚════════════════════════════════════╝");
    xSemaphoreGive(serialMutex);
  }
}

void homing_handleCommand(String cmd) {
  if (cmd == "HOME" || cmd == "HOMING") {
    homing_start();
  } else if (cmd == "RESET" || cmd == "RE") {
    homing_resetEncoder();
  } else if (cmd == "STATUS" || cmd == "ST") {
    if (serialMutex != NULL) {
      xSemaphoreTake(serialMutex, portMAX_DELAY);
      int pos = homing_checkPosition();
      int kiri = analogRead(HOMING_SENSOR_KIRI);
      int kanan = analogRead(HOMING_SENSOR_KANAN);

      Serial.println("\n╔════════════════════════════════════╗");
      Serial.println("║          STATUS HOMING             ║");
      Serial.println("╚════════════════════════════════════╝");

      Serial.print("📊 Posisi: ");
      switch (pos) {
        case ARAH_HOME:
          Serial.println("HOME ✅ (Kiri HIGH, Kanan LOW)");
          break;
        case ARAH_KE_KIRI:
          Serial.println("Magnet di KANAN -> Putar KIRI (Keduanya HIGH)");
          break;
        case ARAH_KE_KANAN:
          Serial.println("Magnet di KIRI -> Putar KANAN (Keduanya LOW)");
          break;
        default:
          Serial.println("GAP (Lanjut arah terakhir)");
          break;
      }

      Serial.print("   Sensor Kiri : ");
      Serial.print(kiri);
      Serial.print(" (");
      if (kiri >= HOMING_SENSOR_HIGH) Serial.print("HIGH");
      else if (kiri <= HOMING_SENSOR_LOW) Serial.print("LOW");
      else Serial.print("MID");
      Serial.println(")");

      Serial.print("   Sensor Kanan: ");
      Serial.print(kanan);
      Serial.print(" (");
      if (kanan >= HOMING_SENSOR_HIGH) Serial.print("HIGH");
      else if (kanan <= HOMING_SENSOR_LOW) Serial.print("LOW");
      else Serial.print("MID");
      Serial.println(")");

      Serial.print("   Homing Active: ");
      Serial.println(homingActive ? "YA" : "TIDAK");
      Serial.print("   Arah Terakhir: ");
      if (lastDirection == 1) Serial.println("KANAN");
      else if (lastDirection == -1) Serial.println("KIRI");
      else Serial.println("STOP");
      Serial.print("   Encoder: ");
      Serial.println(steerAngleDeg, 1);
      Serial.println("════════════════════════════════════\n");
      xSemaphoreGive(serialMutex);
    }
  }
}

void homingTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);
  static unsigned long lastPrint = 0;

  vTaskDelay(pdMS_TO_TICKS(100));

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    homing_calculateRpm();

    if (homingActive) {
      // Pause steering task saat homing
      if (steeringTaskHandle != NULL) {
        vTaskSuspend(steeringTaskHandle);
      }

      homing_moveToHome();

      if (millis() - lastPrint > 1000) {
        lastPrint = millis();
        if (serialMutex != NULL && xSemaphoreTake(serialMutex, 0) == pdTRUE) {
          int pos = homing_checkPosition();
          int kiri = analogRead(HOMING_SENSOR_KIRI);
          int kanan = analogRead(HOMING_SENSOR_KANAN);

          Serial.print("🏠 HOMING | RPM: ");
          Serial.print(currentRpm, 1);
          Serial.print(" | PWM: ");
          Serial.print((int)pidOutput);
          Serial.print(" | Pos: ");
          Serial.print(pos == 0 ? "GAP" : String(pos));
          Serial.print(" | Arah: ");
          Serial.print(lastDirection == 1 ? "KANAN" : (lastDirection == -1 ? "KIRI" : "STOP"));
          Serial.print(" | Kiri: ");
          Serial.print(kiri);
          Serial.print(" | Kanan: ");
          Serial.println(kanan);
          xSemaphoreGive(serialMutex);
        }
      }
    } else {
      // Resume steering task saat homing selesai
      if (steeringTaskHandle != NULL && homingCompleted) {
        vTaskResume(steeringTaskHandle);
        homingCompleted = false;
      }
      homing_stop();
    }
  }
}

#endif  // HOMING_H