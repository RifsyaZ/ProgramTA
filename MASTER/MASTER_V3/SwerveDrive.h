//---------------------- KIRIM PERINTAH KE SEMUA ID UNTUK KE SUDUT DAN RPM RODA ----------------------//
//KIRI DEPAN    ID1 = A1,R1        KANAN DEPAN    ID2 = A2,R2
//KIRI BELAKANG ID4 = A4,R4        KANAN BELAKANG ID2 = A3,R3
void MOV(float A1, float R1, float A2, float R2, float A3, float R3, float A4, float R4) {
  String cmd = "ALL:";
  cmd += String(A1, 1) + "," + String(R1, 1) + ":";
  cmd += String(A2, 1) + "," + String(R2, 1) + ":";
  cmd += String(A3, 1) + "," + String(R3, 1) + ":";
  cmd += String(A4, 1) + "," + String(R4, 1);

  Serial1.println(cmd);
  Serial.print("MOV: ");
  Serial.println(cmd);
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
    Serial1.println(":HOME");
    delay(5);
  }
  Serial.println("HOME ALL");
}

void SwerveDrive(float Vx, float Vy, float Wz, float max_rpm) {
  float target_angle[4];
  float target_rpm[4];

  for (int i = 0; i < 4; i++) {
    float vx_i = Vx - (Wz * module_y[i]);  // ROS: vx_i = Vx - Wz * li_y
    float vy_i = Vy + (Wz * module_x[i]);  // ROS: vy_i = Vy + Wz * li_x

    float speed_ms = sqrtf(vx_i * vx_i + vy_i * vy_i);  // ROS: vi = sqrt(vx_i^2 + vy_i^2)
    float rps = speed_ms / (2.0f * PI * R_WHEEL);
    float rpm = rps * 60.0f;

    float raw_angle = atan2f(vy_i, vx_i);  // ROS: phii = atan2(vy_i, vx_i)

    float prev_ros = 360.0f - prev_angle[i];  // ROS: prev_ros = prev_angle[i]
    if (prev_ros >= 360.0f) prev_ros -= 360.0f;

    float delta = raw_angle - (prev_ros * PI / 180.0f);
    while (delta > PI) delta -= 2 * PI;
    while (delta < -PI) delta += 2 * PI;

    if (fabsf(delta) > PI / 2.0f) {
      raw_angle += PI;
    }

    float angle_deg = 360.0f - (raw_angle * 180.0f / PI);  // ROS: angle_deg = raw_angle * 180/PI
    while (angle_deg < 0) angle_deg += 360.0f;
    while (angle_deg >= 360) angle_deg -= 360.0f;

    target_angle[i] = angle_deg;
    target_rpm[i] = fabsf(rpm);  // ROS: target_rpm[i] = rpm
    prev_angle[i] = angle_deg;
  }

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

  MOV(target_angle[0], target_rpm[0],   // ID1 FL
      target_angle[1], target_rpm[1],   // ID2 FR
      target_angle[3], target_rpm[3],   // ID3 RR
      target_angle[2], target_rpm[2]);  // ID4 RL
}