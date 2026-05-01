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
    }
  }
}

//---------------------- FUNGSI UNTUK MEMERINTAH MCU MPU UNTUK CALIBRATION MPU ----------------------//
void cal() {
  Serial5.println("CAL");
}

//---------------------- Read Fedback Sudut & Pulsa Encoder ----------------------//
void fedback() {
  //---------------------- ID 1----------------------//
  while (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    data.trim();

    if (data.startsWith("STEER:")) {
      String valueStrID1 = data.substring(6);  // Ambil setelah "STEER:"
      Fedback_Angle[0] = valueStrID1.toFloat();
    } else if (data.startsWith("ENC:")) {
      String valueStrID1 = data.substring(4);  // Ambil setelah "ENC:"
      Fedback_Pulse[0] = valueStrID1.toInt();
    }
  }
  //---------------------- ID 2----------------------//
  while (Serial3.available()) {
    String data = Serial3.readStringUntil('\n');
    data.trim();

    if (data.startsWith("STEER:")) {
      String valueStrID2 = data.substring(6);  // Ambil setelah "STEER:"
      Fedback_Angle[1] = valueStrID2.toFloat();
    } else if (data.startsWith("ENC:")) {
      String valueStrID2 = data.substring(4);  // Ambil setelah "ENC:"
      Fedback_Pulse[1] = valueStrID2.toInt();
    }
  }
  //---------------------- ID 3----------------------//
  while (Serial4.available()) {
    String data = Serial4.readStringUntil('\n');
    data.trim();

    if (data.startsWith("STEER:")) {
      String valueStrID3 = data.substring(6);  // Ambil setelah "STEER:"
      Fedback_Angle[2] = valueStrID3.toFloat();
    } else if (data.startsWith("ENC:")) {
      String valueStrID3 = data.substring(4);  // Ambil setelah "ENC:"
      Fedback_Pulse[2] = valueStrID3.toInt();
    }
  }
  //---------------------- ID 4----------------------//
  while (Serial6.available()) {
    String data = Serial6.readStringUntil('\n');
    data.trim();

    if (data.startsWith("STEER:")) {
      String valueStrID4 = data.substring(6);  // Ambil setelah "STEER:"
      Fedback_Angle[3] = valueStrID4.toFloat();
    } else if (data.startsWith("ENC:")) {
      String valueStrID4 = data.substring(4);  // Ambil setelah "ENC:"
      Fedback_Pulse[3] = valueStrID4.toInt();
    }
  }
}

void Debug_odometry() {
  // Serial.print(Fedback_Angle[0]);
  // Serial.print(" ");
  // Serial.print(Fedback_Pulse[0]);

  // Serial.print("        ");

  // Serial.print(Fedback_Angle[1]);
  // Serial.print(" ");
  // Serial.print(Fedback_Pulse[1]);

  // Serial.print("        ");

  // Serial.print(Fedback_Angle[2]);
  // Serial.print(" ");
  // Serial.print(Fedback_Pulse[2]);

  // Serial.print("        ");

  // Serial.print(Fedback_Angle[3]);
  // Serial.print(" ");
  // Serial.print(Fedback_Pulse[3]);

  // Serial.print("        ");

  Serial.print(yaw);

  Serial.println("");
}