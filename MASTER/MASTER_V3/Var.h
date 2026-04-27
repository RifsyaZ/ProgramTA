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
// HardwareSerial Serial4(RX_ID3, TX_ID3);

#define RX_ID4 PC7
#define TX_ID4 PC6
HardwareSerial Serial6(RX_ID4, TX_ID4);


//---------------------- Variabel MPU ----------------------//
float yaw;

// ======================= KONFIGURASI ROBOT =======================
#define PI                3.14159265359f
#define R_WHEEL           0.0225f    // Jari-jari roda (45mm / 2)

// Koordinat modul (meter) - 259.5 / 2
#define L_X               0.12975f
#define L_Y               0.12975f

// Urutan: FL, FR, RL, RR
const float module_x[4] = {  L_X,  L_X, -L_X, -L_X };
const float module_y[4] = {  L_Y, -L_Y,  L_Y, -L_Y };

// Sudut sebelumnya
static float prev_angle[4] = {0, 0, 0, 0};