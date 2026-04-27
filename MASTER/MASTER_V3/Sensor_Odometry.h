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
    Serial.println(data);
  }
}