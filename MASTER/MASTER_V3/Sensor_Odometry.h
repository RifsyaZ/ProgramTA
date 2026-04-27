// ==================== VARIABEL GLOBAL UNTUK DATA SLAVE ID4 ====================
float steer_angle_id4 = 0.0;
long encoder_pulses_id4 = 0;

// ==================== FUNGSI BACA DATA DARI MPU ====================
void MPU1() {
  while (Serial5.available()) {
    String data = Serial5.readStringUntil('\n');
    data.trim();
    if (data == "CAL:START") {
      Serial.println("Sedang melakukan Calibrasi MPU (Tunggu_Sampai_Selesai)");
    } else if (data == "CAL:DONE") {
      Serial.println("Calibrasi MPU Selesai");
    } else {
      yaw = data.toFloat();
      Serial.print("YAW: ");
      Serial.println(yaw);
    }
  }
}

//---------------------- FUNGSI UNTUK MEMERINTAH MCU MPU UNTUK CALIBRATION MPU ----------------------//
void cal() {
  Serial5.println("CAL");
}

void fedback() {
  while (Serial6.available()) {
    String data = Serial6.readStringUntil('\n');
    data.trim();
    
    if (data.startsWith("STEER:")) {
      String valueStr = data.substring(6); // Ambil setelah "STEER:"
      steer_angle_id4 = valueStr.toFloat();
    } else if (data.startsWith("ENC:")) {
      String valueStr = data.substring(4); // Ambil setelah "ENC:"
      encoder_pulses_id4 = valueStr.toInt();
    }
    
    Serial.print(steer_angle_id4); Serial.print(" ");Serial.println(encoder_pulses_id4);

  }
}