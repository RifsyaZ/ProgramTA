void pidcalc(unsigned char type, int sp, int x) {
  if (type == GYRO) {
    int error = 0;
    int lastError = 0;
    int integralError = 0;
    int rateError = 0;
    MPU1();

    if (yaw > x + 1)  error = x - yaw;
    if (yaw >= x && yaw <= x) error = 0;
    if (yaw < x - 1)  error = x - yaw;

    rateError = error - lastError;
    integralError += error;
    lastError = error;

    int kp = 8;//38
    int ki = 2;
    int kd = 5;

    Serial.print("error : "); Serial.println(error);
    Serial.print("rateError : "); Serial.println(rateError);
    Serial.print("integralError : "); Serial.println(integralError);

    int rot_kanan  = 900;
    int angleVal = (int) (error * kp) + (integralError * ki) + (rateError * kd);
    Serial.print("moveVal : "); Serial.println(angleVal);
    int moveLeft  = -900 - (angleVal);
    int moveRight = rot_kanan - angleVal;
    //    moveLeft = constrain(moveLeft, -550, -1000);
    moveRight = constrain(moveRight, 550, 900);

    if (error == 0) {
      wheel(sp, 0, 0);
      Serial.println("LURUS");
    } else if (error > 0) {
      turnn(sp, moveRight);
      Serial.print("moveRight : "); Serial.println(moveRight);
    } else if (error < 0) {
      turnn(sp, moveLeft);
      Serial.print("moveLeft : "); Serial.println(moveLeft);
    }
  }
}
