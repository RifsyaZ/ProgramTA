void CommunicationESP(){
  memcpy(buffer, Fedback_Angle, 16);
  memcpy(buffer + 16, Fedback_Pulse, 16);
  memcpy(buffer + 32, &yaw, 4);

  // Serial.print("KIRIM STM: ");
  // for (int i = 0; i < 36; i++) {
    // Serial.print(buffer[i]); Serial.print(" ");
  // }
  // Serial.println();

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(buffer, 18);
  Wire.endTransmission();

  delay(5);

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(buffer + 18, 18);
  Wire.endTransmission();

  delay(50);

  Wire.requestFrom(SLAVE_ADDR, 32);

  Serial.print("CMD dari ESP: ");
  while (Wire.available()) {
    Serial.println((char)Wire.read());
  }
}