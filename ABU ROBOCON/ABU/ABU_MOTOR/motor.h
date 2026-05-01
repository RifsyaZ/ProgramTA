float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  Serial.print("map Float : "); Serial.println((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
  return (x - in_min)
         * (out_max - out_min)
         / (in_max - in_min)
         + out_min;
}

float constrainPWM(float value) {
  return constrain(value, MIN_PWM, MAX_PWM);
}

void setWheelSpeeds(float v, float omega) {
  // Hitung kecepatan roda menggunakan kinematika differential drive
  float v_right = v + (omega * WHEEL_BASE / 2);
  float v_left = v - (omega * WHEEL_BASE / 2);

  // Konversi kecepatan linier ke kecepatan sudut (rad/s)
  float omega_right = v_right / WHEEL_RADIUS;
  float omega_left = v_left / WHEEL_RADIUS;
  Serial.print("v_right : "); Serial.println(v_right);
  Serial.print("v_left : "); Serial.println(v_left);

  // Skala kecepatan agar tidak mencapai nilai maksimal
  float maxOmega = max(abs(omega_right), abs(omega_left));
  Serial.print("max Omega : "); Serial.println(maxOmega);
  //  float maxOmega = abs(max(omega_right, omega_left));
  if (maxOmega > MAX_PWM) {
    omega_right = omega_right * MAX_PWM / maxOmega;
    omega_left = omega_left * MAX_PWM / maxOmega;
  }

  // Konversi kecepatan sudut ke nilai PWM
  wheelSpeed1 = mapFloat(abs(omega_right), 0, MAX_PWM, MIN_PWM, MAX_PWM);
  wheelSpeed2 = mapFloat(abs(omega_right), 0, MAX_PWM, MIN_PWM, MAX_PWM);
  wheelSpeed3 = mapFloat(abs(omega_left), 0, MAX_PWM, MIN_PWM, MAX_PWM);
  wheelSpeed4 = mapFloat(abs(omega_left), 0, MAX_PWM, MIN_PWM, MAX_PWM);

  // Batasi nilai PWM
  wheelSpeed1 = constrainPWM(wheelSpeed1);
  wheelSpeed2 = constrainPWM(wheelSpeed2);
  wheelSpeed3 = constrainPWM(wheelSpeed3);
  wheelSpeed4 = constrainPWM(wheelSpeed4);

  // Cetak kecepatan roda untuk debugging
  Serial.print("CEK Roda Depan Kiri : "); Serial.println(wheelSpeed1);
  Serial.print("CEK Roda Belakang Kiri : "); Serial.println(wheelSpeed2);
  Serial.print("CEK Roda Depan Kanan : "); Serial.println(wheelSpeed3);
  Serial.print("CEK Roda Belakang Kanan : "); Serial.println(wheelSpeed4);

  // Sesuaikan arah berdasarkan tanda kecepatan
  if (omega_right >= 0) {
    analogWrite(DKI1, wheelSpeed1);
    analogWrite(DKI2, 0);
    analogWrite(BKI1, wheelSpeed2);
    analogWrite(BKI2, 0);
  } else {
    analogWrite(DKI1, 0);
    analogWrite(DKI2, wheelSpeed1);
    analogWrite(BKI1, 0);
    analogWrite(BKI2, wheelSpeed2);
  }

  if (omega_left >= 0) {
    analogWrite(DKA1, wheelSpeed3);
    analogWrite(DKA2, 0);
    analogWrite(BKA1, wheelSpeed4);
    analogWrite(BKA2, 0);
  } else {
    analogWrite(DKA1, 0);
    analogWrite(DKA2, wheelSpeed3);
    analogWrite(BKA1, 0);
    analogWrite(BKA2, wheelSpeed4);
  }
}

void setWheelSpeeds2(float v, float omega, float v_lateral) {
  // Hitung kecepatan roda menggunakan kinematika differential drive
  float v_right = v + (omega * WHEEL_BASE / 2);
  float v_left = v - (omega * WHEEL_BASE / 2);
  Serial.print("v_right : "); Serial.println(v_right);
  Serial.print("v_left : "); Serial.println(v_left);

  // Kombinasikan dengan kecepatan lateral
  float omega_front_right = (v_right - v_lateral) / WHEEL_RADIUS;
  float omega_back_right = (v_right + v_lateral) / WHEEL_RADIUS;
  float omega_front_left = (v_left + v_lateral) / WHEEL_RADIUS;
  float omega_back_left = (v_left - v_lateral) / WHEEL_RADIUS;
  Serial.print("omega_front_right : "); Serial.println(omega_front_right);
  Serial.print("omega_back_right : "); Serial.println(omega_back_right);
  Serial.print("omega_front_left : "); Serial.println(omega_front_left);
  Serial.print("omega_back_left : "); Serial.println(omega_back_left);

  // Skala kecepatan agar tidak mencapai nilai maksimal
  float maxOmega = max(max(abs(omega_front_right), abs(omega_back_right)),
                       max(abs(omega_front_left), abs(omega_back_left)));
  Serial.print("max Omega : "); Serial.println(maxOmega);

  if (maxOmega > MAX_PWM) {
    omega_front_right *= MAX_PWM / maxOmega;
    omega_back_right *= MAX_PWM / maxOmega;
    omega_front_left *= MAX_PWM / maxOmega;
    omega_back_left *= MAX_PWM / maxOmega;
  }

  // Konversi kecepatan sudut ke nilai PWM
  wheelSpeed1 = mapFloat(abs(omega_front_right), 0, MAX_PWM, MIN_PWM, MAX_PWM);
  wheelSpeed2 = mapFloat(abs(omega_back_right), 0, MAX_PWM, MIN_PWM, MAX_PWM);
  wheelSpeed3 = mapFloat(abs(omega_front_left), 0, MAX_PWM, MIN_PWM, MAX_PWM);
  wheelSpeed4 = mapFloat(abs(omega_back_left), 0, MAX_PWM, MIN_PWM, MAX_PWM);

  // Batasi nilai PWM
  wheelSpeed1 = constrainPWM(wheelSpeed1);
  wheelSpeed2 = constrainPWM(wheelSpeed2);
  wheelSpeed3 = constrainPWM(wheelSpeed3);
  wheelSpeed4 = constrainPWM(wheelSpeed4);
  Serial.print("CEK Roda Depan Kiri : "); Serial.println(wheelSpeed1);
  Serial.print("CEK Roda Belakang Kiri : "); Serial.println(wheelSpeed2);
  Serial.print("CEK Roda Depan Kanan : "); Serial.println(wheelSpeed3);
  Serial.print("CEK Roda Belakang Kanan : "); Serial.println(wheelSpeed4);

  // Sesuaikan arah berdasarkan tanda kecepatan
  analogWrite(DKI1, omewga_front_right >= 0 ? wheelSpeed1 : 0);
  analogWrite(DKI2, omega_front_right < 0 ? wheelSpeed1 : 0);
  analogWrite(BKI1, omega_back_right >= 0 ? wheelSpeed2 : 0);
  analogWrite(BKI2, omega_back_right < 0 ? wheelSpeed2 : 0);

  analogWrite(DKA1, omega_front_left >= 0 ? wheelSpeed3 : 0);
  analogWrite(DKA2, omega_front_left < 0 ? wheelSpeed3 : 0);
  analogWrite(BKA1, omega_back_left >= 0 ? wheelSpeed4 : 0);
  analogWrite(BKA2, omega_back_left < 0 ? wheelSpeed4 : 0);
}


//====================JALAN MAJU MUNDUR DAN SERONG====================//
void wheel(int SPD, int sudutZX, int lateral) {
  // Konversi sudut dari derajat ke radian
  float sudutZXRadians = sudutZX * (PI / 180.0f);

  // Tentukan kecepatan linier, sudut, dan lateral
  float v = SPD;
  float omega = 0;
  float v_lateral = lateral;

  if (sudutZX == 0) {  // Maju
    omega = 0;
  } else if (sudutZX == 180) {  // Mundur
    v = -SPD;
    omega = 0;
  } else if (sudutZX > 0 && sudutZX < 90) {  // Serong kiri
    float factor = mapFloat(sudutZX, 1, 90, 1.0f, 0.0f);
    omega = v * factor;
  } else if (sudutZX > 90 && sudutZX < 180) {  // Serong kanan
    float factor = mapFloat(sudutZX, 91, 179, 0.0f, 1.0f);
    omega = -v * factor;
  }

  setWheelSpeeds2(v, omega, v_lateral);
}

//====================PUTER KANAN KIRI====================//
void turnn(int SPD, float turnX) {
  // Hitung kecepatan sudut untuk berbelok
  float turnRadius = abs(turnX) / 1000.0;
  float v = SPD;
  float omega = 0;

  if (turnRadius != 0.0f) {
    omega = v / turnRadius;
    if (turnX > 0) {  // Belok kanan
      setWheelSpeeds(v, omega);
    } else {  // Belok kiri
      setWheelSpeeds(v, -omega);
    }
  } else {
    Serial.println("Radius belok tidak valid.");
  }
}
