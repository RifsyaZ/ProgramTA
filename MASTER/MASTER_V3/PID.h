void PID(unsigned char type, int sp, int Target, float max_rpm) {
  if (type == F_GY) {
    int error = 0;
    static int lastError = 0;    
    static int integralError = 0;
    int rateError = 0;
    
    MPU1();
    fedback();
    Debug_odometry();

    // Deadband ±3°
    if (yaw > Target + 3) {
      error = Target - yaw;
    } else if (yaw < Target - 3) {
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

    int kp = 3;
    int ki = 1;
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
    moveLeft = constrain(moveLeft, -500, -150);
    moveRight = constrain(moveRight, 150, 500);

    if (error == 0) {
      SwervePolar(sp, 0, 0, max_rpm);
      Serial.println("LURUS");
    } else {
      int speedNow = sp;
      if (abs(error) < 10) {
        speedNow = sp / 2;
      }
      
      if (error > 0) {
        turnnSwerve(speedNow, moveRight, max_rpm);
        Serial.print("moveRight : ");
        Serial.println(moveRight);
      } else {
        turnnSwerve(speedNow, moveLeft, max_rpm);
        Serial.print("moveLeft : ");
        Serial.println(moveLeft);
      }
    }
  }
}