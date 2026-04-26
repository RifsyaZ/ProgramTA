#ifndef VAR_GLOBAL_H
#define VAR_GLOBAL_H

#include <Arduino.h>
#include <STM32FreeRTOS.h>

// ==================== PIN DEFINITIONS ====================
// Steering motor (Position control) - BTS7960
#define STEER_ENC_A PB2
#define STEER_ENC_B PB1
#define STEER_RPWM PB8   // TIM4 CH3 - NORMAL PWM
#define STEER_LPWM PB9   // TIM4 CH4 - NORMAL PWM

// Driving motor - BTS7960
#define DRIVE_ENC_A PB6
#define DRIVE_ENC_B PB7
#define DRIVE_RPWM PA7   // TIM3 CH2
#define DRIVE_LPWM PB10  // TIM2 CH3

// LED Indikator
#define LED_PIN PC13

// ==================== STEERING CONSTANTS ====================
#define STEER_PULSE_PER_REV 1813.0
#define STEER_PWM_MAX 100
#define STEER_BASE_PWM 50
#define STEER_MIN_PWM 10

extern const float STEER_MIN_ANGLE;
extern const float STEER_MAX_ANGLE;
extern const float STEER_TOLERANCE;

extern float steer_Kp_normal;
extern float steer_Ki_normal;
extern float steer_Kd_normal;
extern float steer_Kp_dekat;
extern float steer_Ki_dekat;
extern float steer_Kd_dekat;

// ==================== DRIVING CONSTANTS ====================
extern const float DRIVE_PULSE_PER_REV;
extern const float DRIVE_GEARBOX_RATIO;
extern const float DRIVE_WHEEL_DIAMETER_MM;
extern const float DRIVE_MAX_WHEEL_RPM;

extern float drive_Kp;
extern float drive_Ki;
extern float drive_Kd;

// ==================== GLOBAL VARIABLES ====================
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
extern const float drive_filterAlpha;
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

// System variables
extern bool steering_active;
extern bool driving_active;
extern unsigned long command_start_time;
extern float requested_angle;
extern float requested_rpm;

// Queue handles
extern QueueHandle_t steerCommandQueue;
extern QueueHandle_t driveCommandQueue;

// Task handles
extern TaskHandle_t steeringTaskHandle;
extern TaskHandle_t drivingTaskHandle;
extern TaskHandle_t commandTaskHandle;
extern TaskHandle_t displayTaskHandle;

extern SemaphoreHandle_t serialMutex;

// Command structure
typedef struct {
  float angle;
  float rpm;
  bool stop_all;
} SwerveCommand_t;

// Buffers
extern const int STEER_NUM_READINGS;
extern float steer_errorHistory[3];
extern int steer_readIndex;
extern float steer_errorTotal;

extern const float DRIVE_INTEGRAL_MAX;
extern const float DRIVE_INTEGRAL_MIN;
extern const int DRIVE_PWM_MIN;
extern const int DRIVE_PWM_MAX;

extern char inputBuffer[50];
extern int bufferIndex;

#endif