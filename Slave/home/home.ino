#define STEER_RPWM PA0
#define STEER_LPWM PA1
#define STEER_ENC_A PB2
#define STEER_ENC_B PB1

#define Sensor_49E_kiri PA4
#define Sensor_49E_kanan PA5

// ==================== PID CONSTANTS ====================
#define HOMING_PID_KP 1.0
#define HOMING_PID_KI 0.3
#define HOMING_PID_KD 0.1

#define HOMING_PWM_MAX 100
#define HOMING_PWM_MIN 10

// ==================== GLOBAL VARIABLES ====================
volatile int Arah = 0;
volatile bool homingActive = false;

// Encoder variables
volatile int32_t encoderPulses = 0;
volatile int32_t lastEncoderCount = 0;
volatile unsigned long lastEncoderTime = 0;
volatile float currentRPM = 0;
#define PULSE_PER_REV 1813.0

// PID variables
volatile float targetRPM = 0;
volatile float pidError = 0;
volatile float pidPrevError = 0;
volatile float pidIntegral = 0;
volatile float pidOutput = 0;
volatile unsigned long lastPidTime = 0;

// ==================== ENCODER FUNCTIONS ====================
void encoderInterrupt() {
  static int lastEncoded = 0;
  int MSB = digitalRead(STEER_ENC_A);
  int LSB = digitalRead(STEER_ENC_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPulses++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPulses--;

  lastEncoded = encoded;
}

void calculateRPM() {
  unsigned long now = millis();
  if (now - lastEncoderTime >= 50) {
    noInterrupts();
    int32_t deltaPulses = encoderPulses - lastEncoderCount;
    lastEncoderCount = encoderPulses;
    interrupts();

    float dt = (now - lastEncoderTime) / 1000.0;
    lastEncoderTime = now;

    float revolutions = deltaPulses / PULSE_PER_REV;
    currentRPM = revolutions * 60.0 / dt;

    static float filteredRPM = 0;
    filteredRPM = filteredRPM * 0.7 + currentRPM * 0.3;
    currentRPM = filteredRPM;
  }
}

// ==================== MOTOR CONTROL WITH PID ====================
// arahPutar: 1 = kanan, -1 = kiri, 0 = stop
void setMotorPID(int targetRpm, int arahPutar) {
  if (arahPutar == 0) {
    analogWrite(STEER_RPWM, 0);
    analogWrite(STEER_LPWM, 0);
    targetRPM = 0;
    pidIntegral = 0;
    return;
  }

  targetRPM = abs(targetRpm);

  unsigned long now = micros();
  float dt = (now - lastPidTime) / 1000000.0;
  if (dt > 0.05) dt = 0.05;
  if (dt < 0.001) dt = 0.001;

  pidError = targetRPM - currentRPM;

  if (abs(pidError) < 20) {
    pidIntegral += pidError * dt;
    pidIntegral = constrain(pidIntegral, -100, 100);
  } else {
    pidIntegral = 0;
  }

  float derivative = (pidError - pidPrevError) / dt;
  pidOutput = (HOMING_PID_KP * pidError) + (HOMING_PID_KI * pidIntegral) + (HOMING_PID_KD * derivative);
  pidPrevError = pidError;
  lastPidTime = now;

  int pwm = constrain(abs(pidOutput), HOMING_PWM_MIN, HOMING_PWM_MAX);
  Serial.print("PWMNYA =");
  Serial.println(pwm);
  // arahPutar: 1 = kanan, -1 = kiri
  if (arahPutar == 1) {
    // Putar Kanan
    analogWrite(STEER_RPWM, 0);
    analogWrite(STEER_LPWM, pwm);
  } else if (arahPutar == -1) {
    // Putar Kiri
    analogWrite(STEER_RPWM, pwm);
    analogWrite(STEER_LPWM, 0);
  }
}

void stopMotor() {
  setMotorPID(0, 0);
}

// ==================== HOMING LOGIC ====================
int ArahHoming() {
  int kiri = analogRead(Sensor_49E_kiri);
  int kanan = analogRead(Sensor_49E_kanan);

  // kedua sensor sudah mendeteksi magnet berada di titik home maka berhenti
  if (kiri >= 950 && kanan <= 450) {
    Arah = 1;
  }
  // kedua sensor mendeteksi magnet berada di sebelah kanan maka putar motor ke arah kiri
  else if (kiri >= 950 && kanan >= 950) {
    Arah = 2;
  }
  // kedua sensor mendeteksi magnet berada di sebelah kiri maka putar motor ke arah kanan
  else if (kiri <= 450 && kanan <= 450) {
    Arah = 3;
  }
  return Arah;
}

void Home(int rpm) {
  int arah = ArahHoming();
  int targetRpm = constrain(rpm, 10, 100);

  if (arah == 0) {
    // Gap tidak tahu kondisi, pilih putar kiri dulu
    setMotorPID(targetRpm, -1);  // -1 = putar kiri
    Serial.println("Gap Putar Kiri Dulu");
    Serial.print("Arah = ");
    Serial.println(arah);

  } else if (arah == 1) {
    // SUDAH DI TITIK Home
    stopMotor();
    Serial.println("Home");
    Serial.print("Arah = ");
    Serial.println(arah);

  } else if (arah == 2) {
    // BERADA DI KIRI MAKA Putar Kanan
    setMotorPID(targetRpm, 1);  // 1 = putar kanan
    Serial.println("Putar Kanan");
    Serial.print("Arah = ");
    Serial.println(arah);

  } else if (arah == 3) {
    // Berada di kanan maka putar kiri
    setMotorPID(targetRpm, -1);  // -1 = putar kiri
    Serial.println("Putar Kiri");
    Serial.print("Arah = ");
    Serial.println(arah);
  }
}

// ==================== AUTO HOMING ====================
void startHoming() {
  if (homingActive) {
    Serial.println("⚠️ Homing already in progress!");
    return;
  }

  homingActive = true;
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║       STARTING HOMING SEQUENCE     ║");
  Serial.println("╚════════════════════════════════════╝");
}

void runAutoHoming() {
  if (!homingActive) return;

  int arah = ArahHoming();

  if (arah == 1) {
    stopMotor();
    homingActive = false;
    Serial.println("Home");
    Serial.print("Arah = ");
    Serial.println(arah);
    Serial.println("✅ HOMING COMPLETED!");
  } else if (arah == 0) {
    setMotorPID(10, -1);  // Gap - putar kiri
    Serial.println("Gap Putar Kiri Dulu");
    Serial.print("Arah = ");
    Serial.println(arah);
  } else if (arah == 2) {
    setMotorPID(10, 1);  // di kiri - putar kanan
    Serial.println("Putar Kanan");
    Serial.print("Arah = ");
    Serial.println(arah);
  } else if (arah == 3) {
    setMotorPID(10, -1);  // di kanan - putar kiri
    Serial.println("Putar Kiri");
    Serial.print("Arah = ");
    Serial.println(arah);
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000 && homingActive) {
    lastPrint = millis();
    Serial.print("📊 RPM: ");
    Serial.print(currentRPM, 1);
    Serial.print(" | PWM: ");
    Serial.print(pidOutput, 0);
    Serial.print(" | Kiri: ");
    Serial.print(analogRead(Sensor_49E_kiri));
    Serial.print(" | Kanan: ");
    Serial.println(analogRead(Sensor_49E_kanan));
  }
}

// ==================== SETUP & LOOP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(STEER_RPWM, OUTPUT);
  pinMode(STEER_LPWM, OUTPUT);
  pinMode(Sensor_49E_kiri, INPUT);
  pinMode(Sensor_49E_kanan, INPUT);
  pinMode(STEER_ENC_A, INPUT_PULLUP);
  pinMode(STEER_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(STEER_ENC_A), encoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEER_ENC_B), encoderInterrupt, CHANGE);

  lastEncoderTime = millis();
  lastPidTime = micros();

  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║     HOMING WITH PID CONTROL       ║");
  Serial.println("╚════════════════════════════════════╝");
  Serial.println("\nCommands:");
  Serial.println("  HOME       - Start auto homing");
  Serial.println("  STOP       - Emergency stop");
  Serial.println("  STATUS     - Show status");
  Serial.println();
}

void loop() {
  calculateRPM();

  if (homingActive) {
    runAutoHoming();
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "HOME") {
      Arah = 0;
      homingActive = false;
      startHoming();
    } else if (cmd == "STOP") {
      stopMotor();
      homingActive = false;
      Serial.println("⏹️ Emergency stop");
    } else if (cmd == "STATUS") {
      int arah = ArahHoming();
      Serial.println("\n=== STATUS ===");
      Serial.print("Homing Active: ");
      Serial.println(homingActive ? "YES" : "NO");
      Serial.print("Arah: ");
      Serial.println(arah);
      Serial.print("Current RPM: ");
      Serial.println(currentRPM, 1);
      Serial.print("Target RPM: ");
      Serial.println(targetRPM, 1);
      Serial.print("PID Output: ");
      Serial.println(pidOutput, 0);
      Serial.print("Encoder: ");
      Serial.println(encoderPulses);
      Serial.print("Sensor Kiri: ");
      Serial.println(analogRead(Sensor_49E_kiri));
      Serial.print("Sensor Kanan: ");
      Serial.println(analogRead(Sensor_49E_kanan));
      Serial.println("===============\n");
    }
  }

  delay(10);
}