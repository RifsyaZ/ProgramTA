#define RS485_TX_PIN PA9
#define RS485_RX_PIN PA10

HardwareSerial Serial1(RS485_RX_PIN, RS485_TX_PIN);

// ==================== FUNGSI MOV() ====================
// MOV(sudut1, rpm1, sudut2, rpm2, sudut3, rpm3, sudut4, rpm4)
void MOV(float A1, float R1, float A2, float R2, float A3, float R3, float A4, float R4) {
  // Format: ALL:A1,R1:A2,R2:A3,R3:A4,R4
  String cmd = "ALL:";
  cmd += String(A1, 1) + "," + String(R1, 1) + ":";
  cmd += String(A2, 1) + "," + String(R2, 1) + ":";
  cmd += String(A3, 1) + "," + String(R3, 1) + ":";
  cmd += String(A4, 1) + "," + String(R4, 1);
  
  Serial1.println(cmd);
  
  Serial.print("MOV: ");
  Serial.println(cmd);
}

// ==================== FUNGSI PEMBANTU ====================
void STOP_ALL() {
  MOV(0, 0, 0, 0, 0, 0, 0, 0);
  Serial.println("STOP");
}

void HOME_ALL() {
  for (int i = 1; i <= 4; i++) {
    Serial1.print(i);
    Serial1.println(":HOME");
    delay(5);
  }
  Serial.println("HOME ALL");
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  
  delay(1000);
}

// ==================== LOOP ====================
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;
    
    cmd.toUpperCase();
    
    // ===== PERINTAH CEPAT =====
    if (cmd == "MAJU") {
      Serial.println("\n➡️ MAJU");
      MOV(0, 30, 0, 30, 0, 30, 0, 30);
    }
    else if (cmd == "MUNDUR") {
      Serial.println("\n⬅️ MUNDUR");
      MOV(180, 50, 180, 50, 180, 50, 180, 50);
    }
    else if (cmd == "KANAN") {
      Serial.println("\n↔️ STRAFE KANAN");
      MOV(90, 40, 90, 40, 90, 40, 90, 40);
    }
    else if (cmd == "KIRI") {
      Serial.println("\n↔️ STRAFE KIRI");
      MOV(270, 40, 270, 40, 270, 40, 270, 40);
    }
    else if (cmd == "ROTASI") {
      Serial.println("\n🔄 ROTASI");
      MOV(45, 30, 135, 30, 225, 30, 315, 30);
    }
    else if (cmd == "STOP") {
      STOP_ALL();
    }
    else if (cmd == "HOMEALL" || cmd == "HOME ALL") {
      HOME_ALL();
    }
    
    // ===== TEST POLA =====
    else if (cmd == "POLA1") {
      Serial.println("\n🧪 POLA1: Maju -> Mundur -> Stop");
      MOV(0, 50, 0, 50, 0, 50, 0, 50);
      delay(2000);
      MOV(180, 50, 180, 50, 180, 50, 180, 50);
      delay(2000);
      STOP_ALL();
    }
    else if (cmd == "POLA2") {
      Serial.println("\n🧪 POLA2: Rotasi kanan -> Rotasi kiri -> Stop");
      MOV(45, 30, 135, 30, 225, 30, 315, 30);
      delay(3000);
      MOV(315, 30, 225, 30, 135, 30, 45, 30);
      delay(3000);
      STOP_ALL();
    }
    else if (cmd == "POLA3") {
      Serial.println("\n🧪 POLA3: Strafe kanan -> kiri -> Stop");
      MOV(90, 40, 90, 40, 90, 40, 90, 40);
      delay(2000);
      MOV(270, 40, 270, 40, 270, 40, 270, 40);
      delay(2000);
      STOP_ALL();
    }
    else if (cmd == "POLA4") {
      Serial.println("\n🧪 POLA4: Diamond (serong)");
      MOV(45, 40, 45, 40, 45, 40, 45, 40);
      delay(2000);
      MOV(135, 40, 135, 40, 135, 40, 135, 40);
      delay(2000);
      MOV(225, 40, 225, 40, 225, 40, 225, 40);
      delay(2000);
      MOV(315, 40, 315, 40, 315, 40, 315, 40);
      delay(2000);
      STOP_ALL();
    }
    
    // ===== PERINTAH MOV MANUAL =====
    else if (cmd.startsWith("MOV")) {
      String params = cmd.substring(3);  // Buang "MOV"
      params.trim();
      
      // Split berdasarkan spasi
      float A[4], R[4];
      int count = 0;
      int pos = 0;
      
      while (pos < params.length() && count < 4) {
        int spacePos = params.indexOf(' ', pos);
        String pair;
        
        if (spacePos == -1) {
          pair = params.substring(pos);
        } else {
          pair = params.substring(pos, spacePos);
        }
        
        int comma = pair.indexOf(',');
        if (comma > 0) {
          A[count] = pair.substring(0, comma).toFloat();
          R[count] = pair.substring(comma + 1).toFloat();
          count++;
        }
        
        if (spacePos == -1) break;
        pos = spacePos + 1;
      }
      
      if (count == 4) {
        Serial.println("\n📤 MOV MANUAL:");
        MOV(A[0], R[0], A[1], R[1], A[2], R[2], A[3], R[3]);
      } else {
        Serial.println("❌ Format: MOV A1,R1 A2,R2 A3,R3 A4,R4");
        Serial.println("   Contoh: MOV 90,30 90,30 90,30 90,30");
      }
    }
    
    // ===== MANUAL PER ID =====
    else {
      int colon1 = cmd.indexOf(':');
      int colon2 = cmd.indexOf(':', colon1 + 1);
      
      if (colon1 > 0 && colon2 > colon1) {
        Serial.print("📤 ");
        Serial.println(cmd);
        Serial1.println(cmd);
      } else {
        Serial.println("❌ Tidak dikenal!");
        Serial.println("   Coba: MAJU, MUNDUR, KANAN, KIRI, ROTASI, STOP");
        Serial.println("   Atau: MOV A1,R1 A2,R2 A3,R3 A4,R4");
      }
    }
  }
}