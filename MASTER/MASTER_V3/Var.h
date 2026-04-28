//----------------------- Konfigurasi ESP ------------------------//
#define SLAVE_ADDR 0x08
uint8_t buffer[36];

//----------------------- konfigurasi PIN Serial Untuk Komunikasi ke Semua Slave ------------------------//
#define RS485_TX_PIN PA9
#define RS485_RX_PIN PA10
HardwareSerial Serial1(RS485_RX_PIN, RS485_TX_PIN);

#define MPU_RX_PIN PD2
#define MPU_TX_PIN PC12
HardwareSerial Serial5(MPU_RX_PIN, MPU_TX_PIN);

// // Serial untuk feedback dari slave
#define RX_ID1 PA3
#define TX_ID1 PA2
HardwareSerial Serial2(RX_ID1, TX_ID1);

#define RX_ID2 PB11
#define TX_ID2 PB10
HardwareSerial Serial3(RX_ID2, TX_ID2);

// #define RX_ID3 PA1
// #define TX_ID3 PA0
// HardwareSerial Serial4(RX_ID3, TX_ID3); //Sudah terdefini di board

#define RX_ID4 PC7
#define TX_ID4 PC6
HardwareSerial Serial6(RX_ID4, TX_ID4);

//---------------------- Variabel MPU && data Fedback Stering & Pulsa encoder ----------------------//
float yaw;
static float Fedback_Angle[4] = { 0, 0, 0, 0 };
static float Fedback_Pulse[4] = { 0, 0, 0, 0 };

// Variabel untuk command dari ESP32
bool isForward = false;
bool isBackward = false;
bool isLeft = false;
bool isRight = false;
bool isJump = false;
bool isKill = false;
bool isStop = false;

// ======================= KONFIGURASI ROBOT =======================
#define PI 3.14159265359f
#define R_WHEEL 0.0225f  // Jari-jari roda (45mm / 2)

// Koordinat modul (meter) - 259.5 / 2
#define L_X 0.12975f
#define L_Y 0.12975f

// Urutan: FL, FR, RL, RR
const float module_x[4] = { L_X, L_X, -L_X, -L_X };
const float module_y[4] = { L_Y, -L_Y, L_Y, -L_Y };

// Sudut sebelumnya
static float prev_angle[4] = { 0, 0, 0, 0 };

// ======================= KONFIGURASI BUZZER =======================
#define BUZZER_PIN PA5

// Frekuensi 2000-4000Hz biasanya paling keras untuk buzzer pasif kecil
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C4 262
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E4 330
#define NOTE_E5 659
#define NOTE_G5 784
#define NOTE_A5 880

// Frekuensi tinggi khusus untuk "TULULIT" biar lebih nyaring
#define TU_HIGH 2500   // Ganti G4 dengan 2500Hz
#define LU_HIGH 3000   // Ganti B4 dengan 3000Hz
#define LIT_HIGH 3500  // Ganti E5 dengan 3500Hz
#define TET_HIGH 2800  // Ganti A4 dengan 2800Hz

#define TULULIT 1
#define TET_TET 2
#define MPU_Ready 3
#define MPU_BelumReady 4
#define Armed 5

void ModeBuzzer(char Mode) {
  if (Mode == TULULIT) {
    tone(BUZZER_PIN, TU_HIGH, 120);
    delay(130);
    tone(BUZZER_PIN, LU_HIGH, 100);
    delay(110);
    tone(BUZZER_PIN, LIT_HIGH, 80);
    delay(100);
    delay(60);
  } else if (Mode == TET_TET) {
    tone(BUZZER_PIN, TET_HIGH, 80);
    delay(120);
    tone(BUZZER_PIN, TET_HIGH, 80);
    delay(120);
    delay(50);
  } else if (Mode == MPU_Ready) {
    for (int i = 0; i < 3; i++) {
      tone(BUZZER_PIN, LIT_HIGH, 100);
      delay(150);
    }
    delay(80);
  } else if (Mode == MPU_BelumReady) {
    tone(BUZZER_PIN, NOTE_C4, 150);
    delay(300);
    tone(BUZZER_PIN, NOTE_C4, 150);
    delay(500);
  } else if (Mode == Armed) {
    tone(BUZZER_PIN, 3500, 500);
    delay(600);
    noTone(BUZZER_PIN);
  } else {
    Serial.println("Tidak ada nada");
  }
}