void TEST(int Index) {
  unsigned long int Time;
  Time = 0;
  Time = millis();
  MPU1();
  fedback();
  Debug_odometry();
  switch (Index) {
    case 0:
      Time = 0;
      Time = millis();
      while (1) {
        MPU1();
        fedback();
        Debug_odometry();
        SwerveDrive(0.3, 0, 0, 30);
        MPU1();
        Serial.println("Di Case 0");
        if (millis() - Time >= 3000) {
          Index = 0;
          break;
        }
      }
    case 1:
      Time = 0;
      Time = millis();
      MPU1();
      fedback();
      Debug_odometry();
      while (1) {
        MPU1();
        fedback();
        Debug_odometry();
        SwerveDrive(0, 0.3, 0, 0);
        MPU1();
        Serial.println("Di Case 1");
        if (millis() - Time >= 3000) {
          Index = 2;
          break;
        }
      }
    case 2:
      Time = 0;
      Time = millis();
      MPU1();
      fedback();
      Debug_odometry();
      while (1) {
        MPU1();
        fedback();
        Debug_odometry();
        SwerveDrive(0, 0, 0.3, 0);
        MPU1();
        Serial.println("Di Case 2");
        if (millis() - Time >= 3000) {
          Index = 0;
          break;
        }
      }
  }
}