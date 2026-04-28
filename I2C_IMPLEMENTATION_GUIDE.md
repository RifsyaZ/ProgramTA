╔════════════════════════════════════════════════════════════════════════════╗
║          I2C COMMUNICATION: MASTER ↔ ESP32 - IMPLEMENTASI SELESAI          ║
╚════════════════════════════════════════════════════════════════════════════╝

✅ YANG SUDAH DIIMPLEMENTASIKAN:
══════════════════════════════════════════════════════════════════════════════

🔧 File Baru Dibuat:
   • I2C_Comm.h (Library I2C Master & Slave functions)

✏️  File Dimodifikasi:
   • MASTER_V3.ino (Tambah I2C Master + read/send logic)
   • BLE_V1.ino (Tambah I2C Slave + BLE-to-I2C bridge)


📊 FLOW KOMUNIKASI:
══════════════════════════════════════════════════════════════════════════════

1. BLE App → ESP32 (BLE): Kirim command (F/B/L/R/S)
   ↓
2. ESP32 (onWrite callback): Simpan command di ble_command
   ↓
3. Master (I2C_ReadBLECommand): Request command dari ESP32
   ↓ 
4. ESP32 (onI2CRequest): Kirim ble_command ke Master
   ↓
5. Master: Olah command (MAJU/MUNDUR/KIRI/KANAN/STOP)
   ↓
6. Master (I2C_SendOdometry): Kirim sensor data ke ESP32
   ├─ Yaw (dari MPU1)
   ├─ Fedback_Angle[4] (steering angles)
   └─ Fedback_Pulse[4] (encoder pulses)
   ↓
7. ESP32 (onI2CReceive): Terima & simpan odometry data
   ↓
8. (Repeat loop)


🔌 HARDWARE WIRING:
══════════════════════════════════════════════════════════════════════════════

Master (STM32)         ESP32-C3
──────────────────     ─────────
PB6 (SCL) ─────────────→ GPIO8 (SCL)
PB7 (SDA) ─────────────→ GPIO10 (SDA)
GND ──────────────────→ GND

⚠️  WAJIB TAMBAH PULL-UP RESISTOR:
    • 4.7kΩ resistor di SCL (ke +5V)
    • 4.7kΩ resistor di SDA (ke +5V)


🎯 DATA YANG DIKIRIM:
══════════════════════════════════════════════════════════════════════════════

Master ← ESP32 (BLE Command):
  1 byte: 'F' (MAJU), 'B' (MUNDUR), 'L' (KIRI), 'R' (KANAN), 'S' (STOP)

Master → ESP32 (Odometry):
  - Yaw: float (4 bytes) - heading angle dari MPU
  - Angles: 4 × float (16 bytes) - FL, FR, RL, RR steering angle
  - Pulses: 4 × int (8 bytes) - FL, FR, RL, RR encoder pulse
  Total: 29 bytes per transmission


📜 FUNGSI YANG TERSEDIA:
══════════════════════════════════════════════════════════════════════════════

MASTER (STM32):
  I2C_Master_Init()              → Initialize I2C sebagai Master
  char cmd = I2C_ReadBLECommand() → Baca BLE command dari ESP32
  I2C_SendOdometry(...)          → Kirim sensor data ke ESP32

SLAVE (ESP32):
  I2C_Slave_Init()               → Initialize I2C sebagai Slave
  I2C_SetBLECommand(cmd)         → Set command BLE untuk Master (auto dari callback)
  float I2C_GetYaw()             → Get yaw dari Master
  void I2C_GetAngles(float*)     → Get steering angles
  void I2C_GetPulses(int*)       → Get encoder pulses


💻 CONTOH KODE DI MASTER LOOP:
══════════════════════════════════════════════════════════════════════════════

void loop() {
  MPU1();                    // Baca sensor MPU
  fedback();                 // Baca feedback motor
  Debug_odometry();          // Print data ke Serial
  
  // ===== BACA COMMAND BLE DARI ESP32 =====
  char ble_cmd = I2C_ReadBLECommand();
  if (ble_cmd != '0') {
    Serial.print("[I2C RX] Command: ");
    Serial.println(ble_cmd);
    
    // Olah command:
    if (ble_cmd == 'F') {
      // TODO: Eksekusi gerak maju
    }
    else if (ble_cmd == 'B') {
      // TODO: Eksekusi gerak mundur
    }
    // ... dst
  }
  
  // ===== KIRIM ODOMETRY KE ESP32 =====
  I2C_SendOdometry(yaw, 
                   Fedback_Angle[0], Fedback_Angle[1],
                   Fedback_Angle[2], Fedback_Angle[3],
                   (int)Fedback_Pulse[0], (int)Fedback_Pulse[1],
                   (int)Fedback_Pulse[2], (int)Fedback_Pulse[3]);
}


📱 CONTOH KODE DI ESP32 LOOP:
══════════════════════════════════════════════════════════════════════════════

void loop() {
  if (deviceConnected) {
    // BLE command auto diterima di MyCallbacks::onWrite
    // Odometry data auto diterima di onI2CReceive
    
    // Cek data yang diterima dari Master:
    float yaw = I2C_GetYaw();
    float angles[4];
    int pulses[4];
    I2C_GetAngles(angles);
    I2C_GetPulses(pulses);
    
    // Gunakan data untuk:
    // - Tampilkan di dashboard
    // - Processing lokal
    // - Kirim ke aplikasi via BLE notify
    
    delay(100);
  }
}


🔍 CARA TESTING:
══════════════════════════════════════════════════════════════════════════════

1. Upload code ke kedua board:
   • MASTER_V3.ino → STM32
   • BLE_V1.ino → ESP32-C3

2. Buka Serial Monitor (115200 baud) untuk MASTER:
   [MASTER] I2C Ready!
   
   Dan untuk ESP32:
   [I2C] Slave initialized on 0x08

3. Connect BLE dari smartphone
   Master: Client Connected
   ESP32: Client Connected

4. Kirim command dari app (F/B/L/R/S)
   Master Serial: [I2C RX] BLE Command: F
                  MAJU
   
   ESP32 Serial: [BLE RX] Command: F
                 MAJU

5. Verifikasi odometry diterima ESP32:
   ESP32: [I2C RX] Odometry - YAW: 1.23 Angles: 45.1 0.2 ...


⚡ I2C CONFIGURATION:
══════════════════════════════════════════════════════════════════════════════

Address: 0x08
Speed: 400 kHz (Fast mode)
SCL Pin: STM32 PB6 ↔ ESP32 GPIO8
SDA Pin: STM32 PB7 ↔ ESP32 GPIO10


📋 CHECKLIST SEBELUM TESTING:
══════════════════════════════════════════════════════════════════════════════

Hardware:
  ☐ SCL & SDA sudah terhubung dengan benar
  ☐ Pull-up resistor 4.7kΩ sudah dipasang
  ☐ GND sudah terhubung

Software:
  ☐ I2C_Comm.h sudah di folder MASTER_V3/
  ☐ MASTER_V3.ino include "I2C_Comm.h"
  ☐ BLE_V1.ino include <Wire.h>
  ☐ I2C_Master_Init() dipanggil di setup() Master
  ☐ Wire.begin(0x08, 10, 8) dipanggil di setup() ESP32

Compile & Upload:
  ☐ MASTER_V3.ino compile tanpa error
  ☐ BLE_V1.ino compile tanpa error
  ☐ Upload kedua board
  ☐ Serial Monitor menunjukkan "[MASTER] I2C Ready!"
  ☐ Serial Monitor menunjukkan "[I2C] Slave initialized"


🎯 NEXT STEPS:
══════════════════════════════════════════════════════════════════════════════

1. Test komunikasi I2C:
   - Verifikasi command BLE terkirim ke Master
   - Verifikasi odometry diterima ESP32

2. Implementasi motion control:
   - Di Master: ubah if(ble_cmd) dengan actual motor commands
   - Test gerak robot (F/B/L/R/S)

3. Display odometry di app:
   - Di ESP32: kirim data via BLE notify
   - Di app: tampilkan sensor feedback


═════════════════════════════════════════════════════════════════════════════

✨ SIAP UNTUK TESTING!

Implementasi I2C communication sudah lengkap. 
Sekarang tinggal:
1. Upload code
2. Hubungkan hardware
3. Test dengan BLE app
4. Implementasi motion control di Master loop

═════════════════════════════════════════════════════════════════════════════
