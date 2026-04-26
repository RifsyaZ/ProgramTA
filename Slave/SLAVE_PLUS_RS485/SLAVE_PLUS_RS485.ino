/**
 * SWERVE DRIVE CONTROL SYSTEM - PARALLEL EXECUTION
 * Steering dan Driving bekerja BERSAMAAN
 * Dengan komunikasi RS485 untuk multi slave (Auto Direction)
 * 
 * Input via USB: A[angle]R[rpm]  Contoh: A90R50a0r0
 * Input via RS485: [ID]A[angle]R[rpm] Contoh: 1A90R50
 * 
 * LOGIKA ARAH:
 * - Sudut 0-90° dan 270-360° → MAJU (RPM positif)
 * - Sudut 90-270° → MUNDUR (RPM negatif)
 * 
 * FITUR TIMEOUT:
 * - RS485: Auto stop ke 0° dan RPM=0 jika tidak ada perintah 1 detik
 * - USB: Tetap berjalan terus sampai ada perintah baru
 */

#include <Arduino.h>
#include <STM32FreeRTOS.h>

// ==================== PIN DEFINITIONS ====================
// RS485 - Auto switching (hanya butuh RX TX)
// Serial1 sudah didefinisikan secara default untuk STM32
// Default: Serial1(PA10, PA9) - RX=PA10, TX=PA9

// Steering motor (Position control) - BTS7960
#define STEER_ENC_A PB2
#define STEER_ENC_B PB1
#define STEER_ENA PB14
#define STEER_IN1 PB13
#define STEER_IN2 PB12

// Driving motor - BTS7960
#define DRIVE_ENC_A PB6
#define DRIVE_ENC_B PB7
#define DRIVE_RPWM PA7
#define DRIVE_LPWM PB10

// LED Indikator
#define LED_PIN PC13

// ==================== KONFIGURASI SLAVE ====================
#define MY_SLAVE_ID 1  // ID unik tiap slave (1-8)
#define MAX_SLAVE_ID 8

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

// ============== VARIABEL SUMBER PERINTAH ==============
enum CommandSource {
  CMD_SOURCE_NONE,
  CMD_SOURCE_USB,
  CMD_SOURCE_RS485
};

CommandSource current_command_source = CMD_SOURCE_NONE;
unsigned long lastRs485CmdTime = 0;
const unsigned long RS485_TIMEOUT = 1000; // 1 detik timeout untuk RS485

// ============== VARIABEL DEFAULT STOP ==============
const float DEFAULT_STOP_ANGLE = 0.0;  // Posisi default saat timeout (0°)
const float DEFAULT_STOP_RPM = 0.0;    // RPM default saat timeout

// Queue untuk komunikasi antar task
QueueHandle_t steerCommandQueue;
QueueHandle_t driveCommandQueue;
QueueHandle_t rs485TxQueue;  // Queue untuk mengirim data via RS485

// ============== STRUCT UNTUK COMMAND ==============
typedef struct {
  float angle;      // Angle dalam 0-360°
  float rpm;        // RPM yang diminta (selalu positif)
  bool stop_all;
  CommandSource source;  // Sumber perintah
} SwerveCommand_t;

// Struct untuk data yang dikirim via RS485
typedef struct {
  char type;        // 'S'=status, 'D'=data, 'H'=heartbeat, 'E'=error
  float value1;     // angle / error code
  float value2;     // rpm / pwm
  float value3;     // status / ready flag
} Rs485Data_t;

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
TaskHandle_t rs485TaskHandle;
TaskHandle_t telemetryTaskHandle;  // Tambahan untuk GUI

// ==================== MUTEX ====================
SemaphoreHandle_t serialMutex;

// ==================== FUNCTION PROTOTYPES ====================
void steeringTask(void *pvParameters);
void drivingTask(void *pvParameters);
void commandTask(void *pvParameters);
void displayTask(void *pvParameters);
void rs485Task(void *pvParameters);
void telemetryTask(void *pvParameters);  // Tambahan untuk GUI

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

// RS485 Communication functions
void rs485SendData(char type, float val1, float val2, float val3);
void rs485ParseCommand(String cmd);
void rs485SendStatus();
void rs485SendHeartbeat();
void rs485SendError(uint8_t errorCode);

// Telemetry untuk GUI
void sendWheelData(int wheel_id, float angle, float rpm, int pwm_pos, int pwm_spd, bool ready);

// Common functions
void parseCommand(String cmd, Stream &replyPort, CommandSource source);
void printHelp(Stream &port);
void printSystemStatus(Stream &port);
void executeSwerveCommand(float angle, float rpm, CommandSource source);
void checkRs485Timeout();  // Fungsi untuk cek timeout RS485

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);  // USB Serial
  Serial1.begin(115200); // RS485 - sudah default untuk STM32 (PA10=RX, PA9=TX)
  delay(1000);
  
  // Buat queue untuk komunikasi antar task
  steerCommandQueue = xQueueCreate(5, sizeof(SwerveCommand_t));
  driveCommandQueue = xQueueCreate(5, sizeof(SwerveCommand_t));
  rs485TxQueue = xQueueCreate(10, sizeof(Rs485Data_t));
  
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
  Serial.println("║     SWERVE DRIVE CONTROL SYSTEM v9.0      ║");
  Serial.println("║       with RS485 Multi-Slave Support       ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.print("║ SLAVE ID: ");
  Serial.print(MY_SLAVE_ID);
  Serial.println("                                         ║");
  Serial.println("╠════════════════════════════════════════════╣");
  Serial.println("║ USB Format : A[angle]R[rpm]                ║");
  Serial.println("║ RS485 Format: [ID]A[angle]R[rpm]           ║");
  Serial.println("║ Contoh: 1A45R20 untuk slave ID 1           ║");
  Serial.println("╠════════════════════════════════════════════╣");
  Serial.println("║ TIMEOUT: RS485 only (1 detik)              ║");
  Serial.println("║ USB: Continuous (no timeout)               ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
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
    2,                    // Prioritas SEDANG (2)
    &drivingTaskHandle
  );
  
  xTaskCreate(
    commandTask,
    "Command Parser",
    256,
    NULL,
    3,                    // Prioritas TERTINGGI (3)
    &commandTaskHandle
  );
  
  xTaskCreate(
    rs485Task,
    "RS485 Comm",
    256,
    NULL,
    2,                    // Prioritas SEDANG (2)
    &rs485TaskHandle
  );
  
  xTaskCreate(
    displayTask,
    "Display",
    256,
    NULL,
    1,                    // Prioritas RENDAH (1)
    &displayTaskHandle
  );
  
  // Task untuk telemetry GUI (tambahan baru)
  xTaskCreate(
    telemetryTask,
    "Telemetry",
    256,
    NULL,
    1,                    // Prioritas RENDAH (1)
    &telemetryTaskHandle
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
        
        // Update sumber perintah
        current_command_source = receivedCmd.source;
      }
    }
    
    // Cek timeout hanya untuk perintah dari RS485
    if (current_command_source == CMD_SOURCE_RS485) {
      checkRs485Timeout();
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
        
        // Kirim status via RS485 (hanya jika sumber dari RS485)
        if (current_command_source == CMD_SOURCE_RS485) {
          Rs485Data_t statusData;
          statusData.type = 'S';
          statusData.value1 = steerAngleDeg;
          statusData.value2 = drive_wheelRpm;
          statusData.value3 = 1; // Ready
          xQueueSend(rs485TxQueue, &statusData, 0);
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

// ==================== FUNGSI CEK TIMEOUT RS485 ====================
void checkRs485Timeout() {
  // Hanya cek timeout jika sumber perintah dari RS485
  if (current_command_source == CMD_SOURCE_RS485) {
    if (millis() - lastRs485CmdTime > RS485_TIMEOUT) {
      // Timeout terjadi - kembalikan ke posisi default (0°) dan RPM 0
      if (serialMutex != NULL && xSemaphoreTake(serialMutex, 0) == pdTRUE) {
        Serial.println("⚠️ RS485 TIMEOUT - Return to default position (0°)");
        xSemaphoreGive(serialMutex);
      }
      
      // Kirim perintah stop ke posisi default
      SwerveCommand_t stopCmd;
      stopCmd.angle = DEFAULT_STOP_ANGLE;
      stopCmd.rpm = DEFAULT_STOP_RPM;
      stopCmd.stop_all = false;
      stopCmd.source = CMD_SOURCE_RS485;
      
      xQueueSend(steerCommandQueue, &stopCmd, 0);
      xQueueSend(driveCommandQueue, &stopCmd, 0);
      
      // Kirim error via RS485
      Rs485Data_t errorData;
      errorData.type = 'E';
      errorData.value1 = 4; // Error timeout
      errorData.value2 = 0;
      errorData.value3 = 0;
      xQueueSend(rs485TxQueue, &errorData, 0);
      
      // Reset sumber perintah
      current_command_source = CMD_SOURCE_NONE;
    }
  }
}

// ==================== TASK: COMMAND PARSING (USB) ====================
void commandTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // Baca dari USB Serial
    while (Serial.available() > 0) {
      char c = Serial.read();
      
      if (c == '\n') {
        if (bufferIndex > 0) {
          inputBuffer[bufferIndex] = '\0';
          String command = String(inputBuffer);
          command.trim();
          command.toUpperCase();
          
          // Parse untuk USB (tanpa ID) - sumber USB
          parseCommand(command, Serial, CMD_SOURCE_USB);
          
          bufferIndex = 0;
          memset(inputBuffer, 0, sizeof(inputBuffer));
        }
      } else if (c != '\r' && bufferIndex < sizeof(inputBuffer) - 1) {
        inputBuffer[bufferIndex++] = c;
      }
    }
  }
}

// ==================== TASK: RS485 COMMUNICATION ====================
void rs485Task(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);
  
  String buffer = "";
  Rs485Data_t txData;
  unsigned long lastHeartbeatSend = 0;
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // Baca data dari RS485
    while (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\n') {
        buffer.trim();
        if (buffer.length() > 0) {
          // Parse command dengan ID
          rs485ParseCommand(buffer);
          buffer = "";
        }
      } else if (c != '\r') {
        buffer += c;
      }
    }
    
    // Kirim data dari queue (auto switching, langsung kirim)
    if (xQueueReceive(rs485TxQueue, &txData, 0) == pdTRUE) {
      if (txData.type == 'S') { // Status
        Serial1.print(MY_SLAVE_ID);
        Serial1.print("S");
        Serial1.print(txData.value1, 2); // Angle
        Serial1.print(",");
        Serial1.print(txData.value2, 2); // RPM
        Serial1.print(",");
        Serial1.print((int)txData.value3); // Ready flag
        Serial1.println();
      } else if (txData.type == 'H') { // Heartbeat
        Serial1.print(MY_SLAVE_ID);
        Serial1.println("H");
      } else if (txData.type == 'E') { // Error
        Serial1.print(MY_SLAVE_ID);
        Serial1.print("E");
        Serial1.println((int)txData.value1);
      }
    }
    
    // Kirim heartbeat periodik (setiap 2 detik)
    if (millis() - lastHeartbeatSend > 2000) {
      Rs485Data_t hbData;
      hbData.type = 'H';
      hbData.value1 = 0;
      hbData.value2 = 0;
      hbData.value3 = 0;
      xQueueSend(rs485TxQueue, &hbData, 0);
      lastHeartbeatSend = millis();
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
      
      // Tampilkan ID Slave
      Serial.print("SLAVE ID: ");
      Serial.println(MY_SLAVE_ID);
      
      // Tampilkan sumber perintah
      Serial.print("SOURCE: ");
      switch(current_command_source) {
        case CMD_SOURCE_USB:
          Serial.print("USB");
          break;
        case CMD_SOURCE_RS485:
          Serial.print("RS485");
          break;
        default:
          Serial.print("NONE");
          break;
      }
      
      // Tampilkan timer RS485 jika sumber dari RS485
      if (current_command_source == CMD_SOURCE_RS485) {
        Serial.print(" | Timeout: ");
        Serial.print(RS485_TIMEOUT - (millis() - lastRs485CmdTime));
        Serial.print("ms");
      }
      Serial.println();
      
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
      
      // RS485 Status
      Serial.print("RS485: ");
      if (millis() - lastRs485CmdTime < RS485_TIMEOUT) {
        Serial.println("🟢 CONNECTED");
      } else {
        Serial.println("🔴 DISCONNECTED");
      }
      
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

// ==================== TASK: TELEMETRY UNTUK GUI PYTHON ====================
void telemetryTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // Kirim data setiap 100ms
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // Kirim data wheel 1
    sendWheelData(1, 
                  steerAngleDeg,                    // angle
                  drive_wheelRpm,                    // rpm
                  (int)steer_currentOutput,          // pwm posisi (menggunakan steer_currentOutput)
                  (int)drive_pwmOutput,               // pwm speed (menggunakan drive_pwmOutput)
                  steer_targetReached);               // ready status
    
    // Untuk wheel 2,3,4 nanti bisa ditambahkan di sini
    // sendWheelData(2, angle2, rpm2, pwm_pos2, pwm_spd2, ready2);
    // sendWheelData(3, angle3, rpm3, pwm_pos3, pwm_spd3, ready3);
    // sendWheelData(4, angle4, rpm4, pwm_pos4, pwm_spd4, ready4);
  }
}

// ==================== FUNGSI KIRIM DATA KE GUI ====================
void sendWheelData(int wheel_id, float angle, float rpm, int pwm_pos, int pwm_spd, bool ready) {
  // Format: ID:angle,rpm,pwm_pos,pwm_spd,ready
  // Contoh: 1:45.2,19.8,120,150,1
  Serial.print(wheel_id);
  Serial.print(":");
  Serial.print(angle, 1);
  Serial.print(",");
  Serial.print(rpm, 1);
  Serial.print(",");
  Serial.print(pwm_pos);
  Serial.print(",");
  Serial.print(pwm_spd);
  Serial.print(",");
  Serial.println(ready ? 1 : 0);
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

// ==================== RS485 FUNCTIONS ====================
void rs485ParseCommand(String cmd) {
  // Format: [ID]A[angle]R[rpm] atau [ID]COMMAND
  // Contoh: 1A45R20 atau 1STOP atau 1STATUS
  
  if (cmd.length() < 2) return;
  
  // Ambil ID
  int id = cmd.charAt(0) - '0';
  if (id < 1 || id > MAX_SLAVE_ID) return;
  
  // Jika bukan untuk slave ini, ignore
  if (id != MY_SLAVE_ID) return;
  
  // Update waktu command terakhir dari RS485
  lastRs485CmdTime = millis();
  
  // Parse sisa command
  String subCmd = cmd.substring(1);
  
  if (subCmd == "STATUS" || subCmd == "?") {
    // Kirim status via RS485
    Rs485Data_t statusData;
    statusData.type = 'S';
    statusData.value1 = steerAngleDeg;
    statusData.value2 = drive_wheelRpm;
    statusData.value3 = steer_targetReached ? 1 : 0;
    xQueueSend(rs485TxQueue, &statusData, 0);
    return;
  }
  
  if (subCmd == "STOP") {
    // Emergency stop
    SwerveCommand_t stopCmd;
    stopCmd.angle = 0;
    stopCmd.rpm = 0;
    stopCmd.stop_all = false;
    stopCmd.source = CMD_SOURCE_RS485;
    
    xQueueSend(steerCommandQueue, &stopCmd, 0);
    xQueueSend(driveCommandQueue, &stopCmd, 0);
    
    // Kirim konfirmasi
    Rs485Data_t ackData;
    ackData.type = 'S';
    ackData.value1 = steerAngleDeg;
    ackData.value2 = 0;
    ackData.value3 = 0;
    xQueueSend(rs485TxQueue, &ackData, 0);
    
    return;
  }
  
  // Parse format A[angle]R[rpm]
  int aIndex = subCmd.indexOf('A');
  int rIndex = subCmd.indexOf('R');
  
  if (aIndex >= 0 && rIndex > aIndex) {
    String angleStr = subCmd.substring(aIndex + 1, rIndex);
    String rpmStr = subCmd.substring(rIndex + 1);
    
    float angle = angleStr.toFloat();
    float rpm = rpmStr.toFloat();
    
    // Validasi input
    if (angle < 0 || angle > 360 || abs(rpm) > 1000) {
      Rs485Data_t errorData;
      errorData.type = 'E';
      errorData.value1 = 3; // Invalid parameter
      errorData.value2 = 0;
      errorData.value3 = 0;
      xQueueSend(rs485TxQueue, &errorData, 0);
      return;
    }
    
    // Eksekusi command dengan sumber RS485
    executeSwerveCommand(angle, rpm, CMD_SOURCE_RS485);
    
    // Kirim konfirmasi via RS485
    Rs485Data_t ackData;
    ackData.type = 'S';
    ackData.value1 = angle;
    ackData.value2 = rpm;
    ackData.value3 = 1;
    xQueueSend(rs485TxQueue, &ackData, 0);
    
    // Tampilkan di USB juga
    if (serialMutex != NULL) {
      xSemaphoreTake(serialMutex, portMAX_DELAY);
      Serial.print("📡 RS485 CMD: ID=");
      Serial.print(id);
      Serial.print(" Angle=");
      Serial.print(angle);
      Serial.print(" RPM=");
      Serial.println(rpm);
      xSemaphoreGive(serialMutex);
    }
  }
}

// ==================== COMMAND EXECUTION ====================
void executeSwerveCommand(float angle, float rpm, CommandSource source) {
  SwerveCommand_t cmd;
  cmd.angle = angle;
  cmd.rpm = rpm;
  cmd.stop_all = false;
  cmd.source = source;
  
  // Update sumber perintah
  current_command_source = source;
  
  // Jika dari RS485, update timer
  if (source == CMD_SOURCE_RS485) {
    lastRs485CmdTime = millis();
  }
  
  // Kirim command ke steering task
  xQueueSend(steerCommandQueue, &cmd, 0);
  
  // Kirim juga ke driving task
  xQueueSend(driveCommandQueue, &cmd, 0);
  
  if (serialMutex != NULL) {
    xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("\n🔄 EKSEKUSI PERINTAH:");
    Serial.print("   Sumber: ");
    Serial.print(source == CMD_SOURCE_USB ? "USB" : "RS485");
    Serial.print(" | Steering: ");
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

void parseCommand(String cmd, Stream &replyPort, CommandSource source) {
  if (cmd == "HELP" || cmd == "H") {
    printHelp(replyPort);
    return;
  }
  
  if (cmd == "STOP" || cmd == "S") {
    SwerveCommand_t stopCmd;
    stopCmd.angle = 0;
    stopCmd.rpm = 0;
    stopCmd.stop_all = false;
    stopCmd.source = source;
    
    xQueueSend(steerCommandQueue, &stopCmd, 0);
    xQueueSend(driveCommandQueue, &stopCmd, 0);
    
    replyPort.println("⏹️ EMERGENCY STOP - SEMUA MOTOR");
    return;
  }
  
  if (cmd == "STATUS" || cmd == "?") {
    printSystemStatus(replyPort);
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
    
    executeSwerveCommand(angle, rpm, source);
    return;
  }
  
  replyPort.println("❌ Command tidak dikenal! Gunakan HELP");
}

void printHelp(Stream &port) {
  port.println("\n╔════════════════════════════════════════════╗");
  port.println("║            HELP - SWERVE DRIVE             ║");
  port.println("╚════════════════════════════════════════════╝");
  
  port.println("\n📝 FORMAT INPUT (USB):");
  port.println("  A[angle]R[rpm]  - Set steering dan driving");
  port.println("  Contoh: A45R20   -> Steering 45°, Maju 20 RPM");
  port.println("          A225R20  -> Steering 225°, Mundur 20 RPM");
  port.println("          STOP     -> Emergency stop");
  
  port.println("\n📝 FORMAT INPUT (RS485):");
  port.println("  [ID]A[angle]R[rpm] - Dengan ID slave");
  port.println("  Contoh: 1A45R20     -> Untuk slave ID 1");
  port.println("          2A225R20    -> Untuk slave ID 2");
  
  port.println("\n⚙️ LOGIKA ARAH BERDASARKAN SUDUT:");
  port.println("  MAJU   : 0° - 90° dan 270° - 360°");
  port.println("  MUNDUR : 90° - 270°");
  
  port.println("\n⚙️ FITUR TIMEOUT:");
  port.println("  • RS485: Auto stop ke 0° jika tidak ada perintah 1 detik");
  port.println("  • USB  : Tidak ada timeout (berjalan terus)");
  
  port.println("\n📋 PERINTAH LAIN:");
  port.println("  STATUS/? - Tampilkan status");
  port.println("  HELP / H - Tampilkan help");
  
  port.println("\n============================================\n");
}

void printSystemStatus(Stream &port) {
  port.println("\n╔════════════════════════════════════════════╗");
  port.println("║            SYSTEM STATUS                   ║");
  port.println("╚════════════════════════════════════════════╝");
  
  port.print("SLAVE ID: ");
  port.println(MY_SLAVE_ID);
  
  port.print("COMMAND SOURCE: ");
  switch(current_command_source) {
    case CMD_SOURCE_USB:
      port.println("USB");
      break;
    case CMD_SOURCE_RS485:
      port.println("RS485");
      break;
    default:
      port.println("NONE");
      break;
  }
  
  port.println("\n🎯 STEERING:");
  port.print("  Rate: 2ms | Posisi: ");
  port.print(steerAngleDeg, 1);
  port.print("° | Target: ");
  port.print(steerTargetAngle, 1);
  port.print("° (");
  port.print(steerRawTargetAngle, 1);
  port.println("°)");
  
  float steerErr = steer_getCurrentError();
  port.print("  Error: ");
  port.print(steerErr, 2);
  port.print("° | Output: ");
  port.println(steer_currentOutput, 1);
  
  port.print("  Status: ");
  if (!steering_active) port.println("IDLE");
  else if (steer_targetReached) port.println("ON TARGET");
  else port.println("MOVING");
  
  port.println("\n⚡ DRIVING:");
  port.print("  Rate: 20ms | Request: ");
  port.print(drive_wheelRpmTarget_requested, 1);
  port.print(" RPM ");
  port.println(drive_directionForward ? "MAJU" : "MUNDUR");
  
  port.print("  Aktual: ");
  port.print(drive_wheelRpm, 1);
  port.print(" RPM | PWM: ");
  port.println(drive_pwmOutput);
  
  port.print("  Status: ");
  if (!driving_active) port.println("IDLE");
  else port.println("AKTIF");
  
  port.println("\n📡 RS485:");
  port.print("  Koneksi: ");
  if (millis() - lastRs485CmdTime < RS485_TIMEOUT) {
    port.println("TERHUBUNG");
    port.print("  Timeout dalam: ");
    port.print(RS485_TIMEOUT - (millis() - lastRs485CmdTime));
    port.println(" ms");
  } else {
    port.println("TERPUTUS");
  }
  
  port.println("\n============================================\n");
}

void loop() {
  // Empty
}