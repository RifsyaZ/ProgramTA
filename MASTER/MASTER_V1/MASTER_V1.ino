HardwareSerial Serial5(PC7, PC6); // RX, TX

void setup() {
  Serial.begin(115200);
  Serial5.begin(115200);
}

void loop() {
  if (Serial5.available()) {
    String data = Serial5.readStringUntil('\n');
    Serial.println(data);
  }
}