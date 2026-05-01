//---------------------- KIRIM PERINTAH KE SEMUA ID UNTUK KE SUDUT DAN RPM RODA ----------------------//
//KIRI DEPAN    ID1 = A1,R1        KANAN DEPAN    ID2 = A2,R2
//KIRI BELAKANG ID4 = A4,R4        KANAN BELAKANG ID2 = A3,R3
void MOV(float A1, float R1, float A2, float R2, float A3, float R3, float A4, float R4) {
  String cmd = "ALL:";
  cmd += String(A1, 1) + "," + String(R1, 1) + ":";
  cmd += String(A2, 1) + "," + String(R2, 1) + ":";
  cmd += String(A3, 1) + "," + String(R3, 1) + ":";  //String(R3, 1)
  cmd += String(A4, 1) + "," + String(R4, 1);

  Serial.print("DEBUG MOV: ");
  Serial.println(cmd);
  Serial1.println(cmd);
  // Serial.print("A1=");
  // Serial.print(A1, 1);
  // Serial.print(" R1=");
  // Serial.println(R1, 1);
  // Serial.print(" A2=");
  // Serial.print(A2, 1);
  // Serial.print(" R2=");
  // Serial.println(R2, 1);
  // Serial.print(" A3=");
  // Serial.print(A3, 1);
  // Serial.print(" R3=");
  // Serial.println(R3, 1);
  // Serial.print(" A4=");
  // Serial.print(A4, 1);
  // Serial.print(" R4=");
  // Serial.println(R4, 1);
}

//---------------------- SET KE SUDUT 0 DAN RPM RODA 0 ----------------------//
void STOP_ALL() {
  MOV(0, 0, 0, 0, 0, 0, 0, 0);
  Serial.println("STOP");
}

//---------------------- SET KE HOME SATU SATU DELAY 5 DETIK ----------------------//
void HOME_ALL() {
  for (int i = 1; i <= 4; i++) {
    Serial1.print(i);
    delay(10);
    Serial1.print(i);
    delay(10);
    Serial1.println(":HOME");
    delay(5);
  }
  Serial.println("HOME ALL");
}

// Vx = kecepatan translasi robot terhadap sumbu x maju/mundur (m/s)
// Vy = kecepatan translasi robot terhadap sumbu y kiri/kanan (m/s)
// Wz = kecepatan sudut rotasi robot terhadap sumbu Z (rad/s), sama dengan omega
void SwerveDrive(float Vx, float Vy, float Wz, float max_rpm) {
  MPU1();
  fedback();
  // Debug_odometry();
  fedback();
  float target_angle[4];
  float target_rpm[4];

  for (int i = 0; i < 4; i++) {
    float x = module_x[i];
    float y = module_y[i];

    float vx_i = Vx - (Wz * y);  // ROS: vx_i = Vx - Wz * li_y
    float vy_i = Vy + (Wz * x);  // ROS: vy_i = Vy + Wz * li_x

    // Kecepatan total pada modul (resultant) untuk konversi ke putaran roda
    float speed_ms = sqrtf(vx_i * vx_i + vy_i * vy_i);  // ROS: vi = sqrt(vx_i^2 + vy_i^2)
    float rps = speed_ms / (2.0f * PI * R_WHEEL);
    float rpm = rps * 60.0f;

    float raw_angle = atan2f(vy_i, vx_i);                  // ROS: phii = atan2(vy_i, vx_i)
    float angle_deg = 360.0f - (raw_angle * 180.0f / PI);  //float angle_deg = 180.0f - (raw_angle * 180.0f / PI);  //float angle_deg = 360.0f - (raw_angle * 180.0f / PI);  // ROS: angle_deg = raw_angle * 180/PI
    while (angle_deg < 0) angle_deg += 360.0f;
    while (angle_deg >= 360) angle_deg -= 360.0f;

    target_angle[i] = angle_deg;
    target_rpm[i] = fabsf(rpm);  // ROS: target_rpm[i] = rpm
    prev_angle[i] = angle_deg;
  }

  if (max_rpm == 0) {
    // Testing steering angles only: kirim sudut, tetapi rpm drive = 0
    for (int i = 0; i < 4; i++) {
      target_rpm[i] = 0;
    }
  } else {
    float max_abs_rpm = 0;
    for (int i = 0; i < 4; i++) {
      if (target_rpm[i] > max_abs_rpm) {  // ROS: fabsf(target_rpm[i])
        max_abs_rpm = target_rpm[i];
      }
    }
    if (max_abs_rpm > max_rpm) {
      float scale = max_rpm / max_abs_rpm;
      for (int i = 0; i < 4; i++) {
        target_rpm[i] *= scale;
      }
    }
  }

  MOV(target_angle[0], target_rpm[0],   // ID1 FL = index 0
      target_angle[1], target_rpm[1],   // ID2 FR = index 1
      target_angle[3], target_rpm[3],   // ID3 RR = index 3
      target_angle[2], target_rpm[2]);  // ID4 RL = index 2
}

void SwervePolar(float speedRPM, float angle, float rotSpeed, float maxRPM) {
  MPU1();
  fedback();
  Debug_odometry();
  // speedRPM: kecepatan dalam RPM yang diinginkan
  float sudutRad = angle * (PI / 180.0f);
  float Wz = rotSpeed * (PI / 180.0f);

  // Konversi RPM ke m/s
  float speed_ms = (speedRPM * 2.0f * PI * R_WHEEL) / 60.0f;

  float Vx = speed_ms * cos(sudutRad);
  float Vy = -speed_ms * sin(sudutRad);

  SwerveDrive(Vx, Vy, Wz, maxRPM);
}


void turnnSwerve(float SPD_rpm, float turnX, float max_rpm) {
  MPU1();
  fedback();
  Debug_odometry();
  float turnRadius = fabsf(turnX) / 100.0;
  float speed_ms = (SPD_rpm * 2.0f * PI * R_WHEEL) / 60.0f;

  if (turnRadius != 0.0f) {
    float omega = speed_ms / turnRadius;
    if (turnX > 0) {
      SwerveDrive(speed_ms, 0, -omega, max_rpm);  // Kanan //+
    } else {
      SwerveDrive(speed_ms, 0, omega, max_rpm);  // Kiri //-
    }
  } else {
    // SwervePolar(SPD_rpm, 0, 0, max_rpm);  // Lurus
    Serial.println("Radius belok tidak valid.");
  }
}