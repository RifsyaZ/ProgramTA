void PID(unsigned char type, int sp, int Target, float max_rpm) {
  if (type == F_GY) {
    volatile int error = 0;
    int lastError = 0;
    int integralError = 0;
    int rateError = 0;

    MPU1();
    fedback();
    Debug_odometry();

    // Deadband ±3°
    if (yaw > Target + 2) {
      error = Target - yaw;
    } else if (yaw < Target - 2) {
      error = Target - yaw;
    } else {
      error = 0;
      integralError = 0;
    }

    rateError = error - lastError;
    integralError += error;
    lastError = error;

    // Batasi integral windup
    // integralError = constrain(integralError, -200, 200);

    int kp = 19;
    int ki = 20;
    int kd = 10;

    Serial.print("error : ");
    Serial.println(error);
    Serial.print("rateError : ");
    Serial.println(rateError);
    Serial.print("integralError : ");
    Serial.println(integralError);

    int angleVal = (error * kp) + (integralError * ki) + (rateError * kd);

    Serial.print("moveVal : ");
    Serial.println(angleVal);

    int moveLeft = -900 - angleVal;
    int moveRight = 900 - angleVal;
    moveLeft = constrain(moveLeft, -500, -20);
    moveRight = constrain(moveRight, 20, 500);

    if (error == 0) {
      SwervePolar(sp, 0, 0, max_rpm);
      Serial.println("LURUS");
    } else if (yaw > Target + 60) {
      int spNew = sp / 2;
      Serial.print("Speed NEW");
      Serial.println(spNew);
      turnnSwerve(spNew, -5, max_rpm);
    } else if (yaw < Target - 60) {
      int spNew = sp / 2;
      Serial.print("Speed NEW");
      Serial.println(spNew);
      turnnSwerve(spNew, 5, max_rpm);
    } else {
      if (error > 0) {
        turnnSwerve(sp, moveRight, max_rpm);
        Serial.print("moveRight : ");
        Serial.println(moveRight);
      } else {
        turnnSwerve(sp, moveLeft, max_rpm);
        Serial.print("moveLeft : ");
        Serial.println(moveLeft);
      }
    }
  }
}