//==============================GYRO==============================//
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

float pitch, roll;
int yaw;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

//==============================PID==============================//
#define GYRO 1
#define BALL 2
#define SILO 3

//==============================MOTOR DC BASE INVERS==============================//
#define PI 3.14159265358979323846
#define WHEEL_BASE 0.27 // Jarak antara roda (dalam meter)
#define WHEEL_RADIUS 5 // Radius roda (dalam millimeter)

#define MIN_PWM 30
#define MAX_PWM 255

float wheelSpeed1, wheelSpeed2, wheelSpeed3, wheelSpeed4;

//==============================MOTOR DC BASE PIN==============================//
//=====Motor kiri
#define DKI1 3
#define DKI2 5
#define BKI1 8
#define BKI2 6

//=====Motor kanan
#define DKA1 9
#define DKA2 7
#define BKA1 44
#define BKA2 46

void MotorPins() {
  pinMode(DKA1, OUTPUT); pinMode(DKA2, OUTPUT); pinMode(DKI1, OUTPUT); pinMode(DKI2, OUTPUT);
  pinMode(BKA2, OUTPUT); pinMode(BKA2, OUTPUT); pinMode(BKI1, OUTPUT); pinMode(BKI2, OUTPUT);
}
