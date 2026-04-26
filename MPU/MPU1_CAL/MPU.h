// Fungsi untuk menyimpan offset ke EEPROM
void saveCalibrationToEEPROM() {
  EEPROM.put(EEPROM_AX_ADDR, cal_ax_offset);
  EEPROM.put(EEPROM_AY_ADDR, cal_ay_offset);
  EEPROM.put(EEPROM_AZ_ADDR, cal_az_offset);
  EEPROM.put(EEPROM_GX_ADDR, cal_gx_offset);
  EEPROM.put(EEPROM_GY_ADDR, cal_gy_offset);
  EEPROM.put(EEPROM_GZ_ADDR, cal_gz_offset);
  
  // Tandai bahwa sudah pernah kalibrasi
  EEPROM.write(EEPROM_CALIBRATED_FLAG, 0xAA);
  
  Serial.println("Calibration data saved to EEPROM!");
}

// Fungsi untuk membaca offset dari EEPROM
void loadCalibrationFromEEPROM() {
  // Cek apakah sudah pernah kalibrasi
  if (EEPROM.read(EEPROM_CALIBRATED_FLAG) == 0xAA) {
    EEPROM.get(EEPROM_AX_ADDR, cal_ax_offset);
    EEPROM.get(EEPROM_AY_ADDR, cal_ay_offset);
    EEPROM.get(EEPROM_AZ_ADDR, cal_az_offset);
    EEPROM.get(EEPROM_GX_ADDR, cal_gx_offset);
    EEPROM.get(EEPROM_GY_ADDR, cal_gy_offset);
    EEPROM.get(EEPROM_GZ_ADDR, cal_gz_offset);
    
    Serial.println("Calibration data loaded from EEPROM:");
    Serial.print("Accel: "); Serial.print(cal_ax_offset); Serial.print(", "); 
    Serial.print(cal_ay_offset); Serial.print(", "); Serial.println(cal_az_offset);
    Serial.print("Gyro: "); Serial.print(cal_gx_offset); Serial.print(", "); 
    Serial.print(cal_gy_offset); Serial.print(", "); Serial.println(cal_gz_offset);
  } else {
    // Default 0 jika belum pernah kalibrasi
    cal_ax_offset = cal_ay_offset = cal_az_offset = 0;
    cal_gx_offset = cal_gy_offset = cal_gz_offset = 0;
    Serial.println("No calibration data found in EEPROM. Using default offsets (0).");
  }
}

void setMPU() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  
  // Load kalibrasi dari EEPROM saat startup
  loadCalibrationFromEEPROM();
  
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // Gunakan hasil kalibrasi dari EEPROM
  mpu.setXAccelOffset(cal_ax_offset);
  mpu.setYAccelOffset(cal_ay_offset);
  mpu.setZAccelOffset(cal_az_offset);
  mpu.setXGyroOffset(cal_gx_offset);
  mpu.setYGyroOffset(cal_gy_offset);
  mpu.setZGyroOffset(cal_gz_offset);
  
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void MPU1() {
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yaw = (ypr[0] * 180 / M_PI) + 180;
    pitch = (ypr[1] * 180 / M_PI);
    roll = (ypr[2] * 180 / M_PI);
    Serial.print("YAW: ");
    Serial.print(yaw);
    Serial.print("\tPITCH :");
    Serial.print(pitch);
    Serial.print("\tROLL :");
    Serial.print(roll);
    Serial.println();
#endif
  }
}

void runCalibration() {
  Serial.println("\n========================================");
  Serial.println("     MPU6050 CALIBRATION STARTED");
  Serial.println("========================================");
  Serial.println("Place MPU6050 on a FLAT HORIZONTAL surface.");
  Serial.println("DO NOT MOVE the sensor during calibration!");

  // Reset offsets to zero untuk kalibrasi
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  int buffersize = 1000;
  int acel_deadzone = 5;
  int giro_deadzone = 1;

  int16_t ax, ay, az, gx, gy, gz;
  long mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;

  // Fungsi meansensors
  auto meansensors = [&]() {
    long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
    while (i < (buffersize + 101)) {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      if (i > 100 && i <= (buffersize + 100)) {
        buff_ax += ax;
        buff_ay += ay;
        buff_az += az;
        buff_gx += gx;
        buff_gy += gy;
        buff_gz += gz;
      }
      i++;
      delay(2);
    }
    mean_ax = buff_ax / buffersize;
    mean_ay = buff_ay / buffersize;
    mean_az = buff_az / buffersize;
    mean_gx = buff_gx / buffersize;
    mean_gy = buff_gy / buffersize;
    mean_gz = buff_gz / buffersize;
  };

  Serial.println("Reading sensor data...");
  meansensors();

  cal_ax_offset = -mean_ax / 8;
  cal_ay_offset = -mean_ay / 8;
  cal_az_offset = (16384 - mean_az) / 8;
  cal_gx_offset = -mean_gx / 4;
  cal_gy_offset = -mean_gy / 4;
  cal_gz_offset = -mean_gz / 4;

  Serial.println("Fine-tuning offsets...");
  int iteration = 0;
  while (1) {
    iteration++;
    int ready = 0;
    mpu.setXAccelOffset(cal_ax_offset);
    mpu.setYAccelOffset(cal_ay_offset);
    mpu.setZAccelOffset(cal_az_offset);
    mpu.setXGyroOffset(cal_gx_offset);
    mpu.setYGyroOffset(cal_gy_offset);
    mpu.setZGyroOffset(cal_gz_offset);

    meansensors();
    
    if (iteration % 5 == 0) {
      Serial.print(".");  // Progress indicator
    }

    if (abs(mean_ax) <= acel_deadzone) ready++;
    else cal_ax_offset -= mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone) ready++;
    else cal_ay_offset -= mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else cal_az_offset += (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) ready++;
    else cal_gx_offset -= mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone) ready++;
    else cal_gy_offset -= mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone) ready++;
    else cal_gz_offset -= mean_gz / (giro_deadzone + 1);

    if (ready == 6) break;
  }

  Serial.println("\n\n========================================");
  Serial.println("       CALIBRATION FINISHED!");
  Serial.println("========================================");
  Serial.print("Accel Offsets: ");
  Serial.print(cal_ax_offset); Serial.print(", ");
  Serial.print(cal_ay_offset); Serial.print(", ");
  Serial.println(cal_az_offset);
  Serial.print("Gyro Offsets:  ");
  Serial.print(cal_gx_offset); Serial.print(", ");
  Serial.print(cal_gy_offset); Serial.print(", ");
  Serial.println(cal_gz_offset);
  
  // Simpan ke EEPROM
  saveCalibrationToEEPROM();
  delay(1000);
  setMPU();
}