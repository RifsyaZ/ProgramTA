#ifndef VAR_GLOBAL_H
#define VAR_GLOBAL_H

#include <Arduino.h>
#include <STM32FreeRTOS.h>

// ==================== PIN DEFINITIONS ====================
#define STEER_ENC_A PB2
#define STEER_ENC_B PB1
#define STEER_RPWM PB8
#define STEER_LPWM PB9

#define DRIVE_ENC_A PB6
#define DRIVE_ENC_B PB7
#define DRIVE_RPWM PA7
#define DRIVE_LPWM PB10

#define LED_PIN PC13

// ==================== CONSTANTS (menggunakan #define, bukan variabel) ====================
#define STEER_PULSE_PER_REV 1813.0
#define STEER_PWM_MAX 255
#define STEER_BASE_PWM 120
#define STEER_MIN_PWM 10
#define STEER_NUM_READINGS 3
#define STEER_MIN_ANGLE -90.0
#define STEER_MAX_ANGLE 90.0
#define STEER_TOLERANCE 0.5

#define DRIVE_PULSE_PER_REV 14.0
#define DRIVE_GEARBOX_RATIO 6.77
#define DRIVE_WHEEL_DIAMETER_MM 82.0
#define DRIVE_MAX_WHEEL_RPM 800.0
#define DRIVE_INTEGRAL_MAX 500.0
#define DRIVE_INTEGRAL_MIN -500.0
#define DRIVE_PWM_MIN 15
#define DRIVE_PWM_MAX 255
#define drive_filterAlpha 0.2

// ==================== GLOBAL VARIABLES (extern - deklarasi) ====================
// Steering variables
extern volatile int32_t steerEncoderCount;
extern int steerLastEncoded;
extern float steerAngleDeg;
extern float steerTargetAngle;
extern float steerRawTargetAngle;
extern float steerTargetSpeed;
extern float steer_lastError;
extern float steer_integral;
extern float steer_prevError;
extern float steer_currentOutput;
extern unsigned long steer_lastTime;
extern bool steer_targetReached;
extern bool steer_isLandingMode;
extern unsigned long steer_landingStartTime;
extern float steer_lastErrorForLanding;
extern float steer_Kp_normal;
extern float steer_Ki_normal;
extern float steer_Kd_normal;
extern float steer_Kp_dekat;
extern float steer_Ki_dekat;
extern float steer_Kd_dekat;

// Steering buffers
extern float steer_errorHistory[3];
extern int steer_readIndex;
extern float steer_errorTotal;

// Driving variables
extern volatile int32_t driveEncoderCount;
extern volatile uint32_t drive_lastInterruptTime;
extern int32_t drive_lastPosition;
extern uint32_t drive_lastRpmTime;
extern float drive_motorRpm;
extern float drive_wheelRpm;
extern float drive_wheelRpmTarget;
extern float drive_wheelRpmTarget_requested;
extern float drive_motorRpmTarget;
extern float drive_rpmFiltered;
extern bool drive_directionForward;
extern float drive_requestedRpm;
extern float drive_integral;
extern float drive_prevError;
extern uint32_t drive_lastPidTime;
extern float drive_pidOutput;
extern int drive_pwmOutput;
extern bool drive_motorRunning;
extern bool drive_direction;
extern bool drive_encoderReverse;
extern float drive_Kp;
extern float drive_Ki;
extern float drive_Kd;

// System variables
extern bool steering_active;
extern bool driving_active;
extern unsigned long command_start_time;
extern float requested_angle;
extern float requested_rpm;

// Queue & Semaphore handles
extern QueueHandle_t steerCommandQueue;
extern QueueHandle_t driveCommandQueue;
extern SemaphoreHandle_t serialMutex;

// Task handles
extern TaskHandle_t steeringTaskHandle;
extern TaskHandle_t drivingTaskHandle;
extern TaskHandle_t commandTaskHandle;
extern TaskHandle_t displayTaskHandle;

// Buffers
extern char inputBuffer[50];
extern int bufferIndex;

// ==================== COMMAND STRUCTURE ====================
typedef struct {
  float angle;
  float rpm;
  bool stop_all;
} SwerveCommand_t;

// ==================== DEFINISI VARIABEL (hanya jika _VAR_GLOBAL_IMPLEMENTATION_ didefinisikan) ====================
#ifdef _VAR_GLOBAL_IMPLEMENTATION_

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
float steer_Kp_normal = 2.5;
float steer_Ki_normal = 0.0;
float steer_Kd_normal = 1.2;
float steer_Kp_dekat = 0.5;
float steer_Ki_dekat = 0.0;
float steer_Kd_dekat = 2.0;

// Steering buffers
float steer_errorHistory[3] = {0};
int steer_readIndex = 0;
float steer_errorTotal = 0;

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
float drive_Kp = 0.05;
float drive_Ki = 0.5;
float drive_Kd = 0.001;

// System variables
bool steering_active = false;
bool driving_active = false;
unsigned long command_start_time = 0;
float requested_angle = 0;
float requested_rpm = 0;

// Queue & Semaphore
QueueHandle_t steerCommandQueue;
QueueHandle_t driveCommandQueue;
SemaphoreHandle_t serialMutex;

// Task handles
TaskHandle_t steeringTaskHandle;
TaskHandle_t drivingTaskHandle;
TaskHandle_t commandTaskHandle;
TaskHandle_t displayTaskHandle;

// Buffers
char inputBuffer[50];
int bufferIndex = 0;

#endif // _VAR_GLOBAL_IMPLEMENTATION_

#endif // VAR_GLOBAL_H