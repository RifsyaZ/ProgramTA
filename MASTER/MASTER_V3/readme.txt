// void SwerveDrive(float Vx, float Vy, float Wz, float max_rpm) {
//   float target_angle[4];
//   float target_rpm[4];

//   for (int i = 0; i < 4; i++) {
//     float x = module_x[i];
//     float y = module_y[i];

//     float vx_i = Vx - (Wz * y);  // ROS: vx_i = Vx - Wz * li_y
//     float vy_i = Vy + (Wz * x);  // ROS: vy_i = Vy + Wz * li_x

//     // Kecepatan total pada modul (resultant) untuk konversi ke putaran roda
//     float speed_ms = sqrtf(vx_i * vx_i + vy_i * vy_i);  // ROS: vi = sqrt(vx_i^2 + vy_i^2)
//     float rps = speed_ms / (2.0f * PI * R_WHEEL);
//     float rpm = rps * 60.0f;

//     float raw_angle = atan2f(vy_i, vx_i);  // ROS: phii = atan2(vy_i, vx_i)

//     float prev_ros = 360.0f - prev_angle[i];  // ROS: prev_ros = prev_angle[i]
//     if (prev_ros >= 360.0f) prev_ros -= 360.0f;

//     float delta = raw_angle - (prev_ros * PI / 180.0f);
//     while (delta > PI) delta -= 2 * PI;
//     while (delta < -PI) delta += 2 * PI;

//     if (fabsf(delta) > PI / 2.0f) {
//       raw_angle += PI;
//     }

//     float angle_deg = 360.0f - (raw_angle * 180.0f / PI);  // ROS: angle_deg = raw_angle * 180/PI
//     while (angle_deg < 0) angle_deg += 360.0f;
//     while (angle_deg >= 360) angle_deg -= 360.0f;

//     target_angle[i] = angle_deg;
//     target_rpm[i] = fabsf(rpm);  // ROS: target_rpm[i] = rpm
//     prev_angle[i] = angle_deg;
//   }

//   float max_abs_rpm = 0;
//   for (int i = 0; i < 4; i++) {
//     if (target_rpm[i] > max_abs_rpm) {  // ROS: fabsf(target_rpm[i])
//       max_abs_rpm = target_rpm[i];
//     }
//   }
//   if (max_abs_rpm > max_rpm) {
//     float scale = max_rpm / max_abs_rpm;
//     for (int i = 0; i < 4; i++) {
//       target_rpm[i] *= scale;
//     }
//   }

//   MOV(target_angle[0], target_rpm[0],   // ID1 FL
//       target_angle[1], target_rpm[1],   // ID2 FR
//       target_angle[3], target_rpm[3],   // ID3 RR
//       target_angle[2], target_rpm[2]);  // ID4 RL
// }

// KODE DIBAWHA BELUM DI UJI //
// MOV_Radius: gerak melingkar dengan body mengikuti arah radius
void MOV_Radius(float Vx, float Vy, float max_rpm, float radius) {
  float used_wz = 0.0f;
  if (radius != 0.0f) {
    float V = sqrtf(Vx * Vx + Vy * Vy);
    if (V > 0.0f) {
      used_wz = (radius > 0.0f ? 1.0f : -1.0f) * (V / fabsf(radius));
    }
  }

  SwerveDrive(Vx, Vy, used_wz, max_rpm);
}

// MOV_CurveFixedHeading: heading robot tetap, path mengikuti kurva dengan kecepatan yang berubah setiap loop.
// Panggil fungsi ini berulang di loop utama agar robot mempertahankan radius yang diinginkan.
// Contoh: MOV_CurveFixedHeading(0.3f, 0.0f, 100.0f, 1.0f);
void MOV_CurveFixedHeading(float Vx, float Vy, float max_rpm, float radius) {
  float V = sqrtf(Vx * Vx + Vy * Vy);
  if (V <= 0.0f || radius == 0.0f) {
    SwerveDrive(Vx, Vy, 0.0f, max_rpm);
    return;
  }

  static float curve_phase = 0.0f;
  static unsigned long last_curve_time = 0;
  static float last_radius = 0.0f;
  static float last_speed = 0.0f;

  unsigned long now = millis();
  if (last_curve_time == 0 || last_radius != radius || fabsf(last_speed - V) > 0.001f) {
    curve_phase = atan2f(Vy, Vx);
    last_curve_time = now;
    last_radius = radius;
    last_speed = V;
  }

  float dt = (now - last_curve_time) * 0.001f;
  if (dt > 0.0f) {
    float direction = (radius > 0.0f) ? 1.0f : -1.0f;
    curve_phase += direction * (V / fabsf(radius)) * dt;
    last_curve_time = now;
  }

  float used_vx = V * cosf(curve_phase);
  float used_vy = V * sinf(curve_phase);

  SwerveDrive(used_vx, used_vy, 0.0f, max_rpm);
}