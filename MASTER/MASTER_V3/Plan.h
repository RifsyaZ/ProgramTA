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

void JoyBLE() {
  if (W && D) {
    Serial.println("MAJU KANAN");
    SwerveDrive(0.3, -0.3, 0, 30);
  } else if (W && A) {
    Serial.println("MAJU Kiri");
    SwerveDrive(0.3, 0.3, 0, 30);
  } else if (S && D) {
    Serial.println("Mundur KANAN");
    SwerveDrive(-0.3, -0.3, 0, 30);
  } else if (S && A) {
    Serial.println("Mundur Kiri");
    SwerveDrive(-0.3, 0.3, 0, 30);
  } else if (W) {
    Serial.println("MAJU");
    SwerveDrive(0.3, 0, 0, 30);
  } else if (S) {
    Serial.println("Mundur");
    SwerveDrive(-0.3, 0, 0, 30);
  } else if (A) {
    Serial.println("kiri");
    SwerveDrive(0, 0.3, 0, 30);
  } else if (D) {
    Serial.println("kanan");
    SwerveDrive(0, -0.3, 0, 30);
  } else if (J) {
    SwerveDrive(0, 0, 0.3, 30);
    Serial.println("CCW");
  } else if (K) {
    SwerveDrive(0, 0, -0.3, 30);
    Serial.println("CW");
  } else if (STOP) {
    STOP_ALL();
    Serial.println("Stop");
  } else if (P) {
    STOP_ALL();
    Serial.println("Stop");
  } else if (H) {
    HOME_ALL();
    Serial.println("HOMING");
  } else if (P1) {
    ModeBuzzer(TULULIT);
  } else if (P2) {
    ModeBuzzer(Armed);
  }
}