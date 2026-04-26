#include <EEPROM.h>

//==============================MPU==============================//
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2
#define MPU_ADDR 0x68 // MPU6050 I2C address
#define PWR_MGMT_1 0x6B // Power management register address

// Alamat EEPROM untuk menyimpan offset
#define EEPROM_AX_ADDR 0
#define EEPROM_AY_ADDR 2
#define EEPROM_AZ_ADDR 4
#define EEPROM_GX_ADDR 6
#define EEPROM_GY_ADDR 8
#define EEPROM_GZ_ADDR 10
#define EEPROM_CALIBRATED_FLAG 20  // Flag untuk mengecek apakah sudah pernah kalibrasi

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;           // [w, x, y, z]
VectorInt16 aa;         // [x, y, z]
VectorInt16 aaReal;     // [x, y, z]
VectorInt16 aaWorld;    // [x, y, z]
VectorFloat gravity;    // [x, y, z]
float ypr[3];           // [yaw, pitch, roll]

uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

float yaw, pitch, roll;

// Variabel untuk menyimpan hasil kalibrasi
int16_t cal_ax_offset, cal_ay_offset, cal_az_offset;
int16_t cal_gx_offset, cal_gy_offset, cal_gz_offset;