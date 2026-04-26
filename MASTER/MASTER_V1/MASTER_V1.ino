#define RS485_TX_PIN PA9
#define RS485_RX_PIN PA10

HardwareSerial RS485(RS485_RX_PIN, RS485_TX_PIN);

void setup() {
  Serial.begin(115200);
  RS485.begin(115200);
  
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║     MASTER - MODUL RS485 WAVESHARE         ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("PA9(TX) -> RXD modul");
  Serial.println("PA10(RX) -> TXD modul");
  Serial.println("\nKetik perintah lalu Enter\n");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() > 0) {
      Serial.print("📤 TX: ");
      Serial.println(cmd);
      RS485.println(cmd);
    }
  }
  
  while (RS485.available()) {
    String resp = RS485.readStringUntil('\n');
    resp.trim();
    
    if (resp.length() > 0) {
      Serial.print("📥 RX: ");
      Serial.println(resp);
    }
  }
  
  delay(10);
}