#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <Wire.h>

#include "var.h"
#include "gyro.h"
#include "motor.h"
#include "PID.h"
#include "set.h"

void setup() {
  Serial.begin(115200);
  setMPU();
  ReSePins();
  MotorPins();
}

void loop() {
  //  ultrasonic();
  //  MPU1();
  if (digitalRead(14) == LOW) {
    Serial.println("START P1");
    plan();
  } else if (digitalRead(16) == LOW) {
    Serial.println("START P2");
    plan2();
  }
  //  camSiloCheck(5);
  //  pidcalcALL(300, 0, 16);
  //  turnn(200, 1200);
  //  wheel(150, 90, 0);
  //  ultraCheck(200, 180, 135);
  //  gyroCheck(10, -90);
}
