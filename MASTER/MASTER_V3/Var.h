//----------------------- konfigurasi PIN Serial Untuk Komunikasi ke Semua Slave ------------------------//
#define RS485_TX_PIN PA9
#define RS485_RX_PIN PA10
HardwareSerial Serial1(RS485_RX_PIN, RS485_TX_PIN);

#define MPU_RX_PIN PD2
#define MPU_TX_PIN PC12
HardwareSerial Serial5(MPU_RX_PIN, MPU_TX_PIN);


//---------------------- Variabel MPU ----------------------//
float yaw;

// ======================= KONFIGURASI ROBOT =======================
#define PI                3.14159265359f
#define R_WHEEL           0.0225f    // Jari-jari roda (45mm / 2)

#define FOLLOW_RADIUS     'R'        // body mengikuti arah radius
#define FOLLOW_DEFINE     'D'        // heading robot tetap terjaga, hanya translasi

// Koordinat modul (meter) - 259.5 / 2
#define L_X               0.12975f
#define L_Y               0.12975f

// Urutan: FL, FR, RL, RR
const float module_x[4] = {  L_X,  L_X, -L_X, -L_X };
const float module_y[4] = {  L_Y, -L_Y,  L_Y, -L_Y };

// Sudut sebelumnya
static float prev_angle[4] = {0, 0, 0, 0};