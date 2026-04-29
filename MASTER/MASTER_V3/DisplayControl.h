void parseCommand(String cmd) {
  // Reset semua flag
  W = false;
  S = false;
  A = false;
  D = false;
  J = false;
  K = false;
  P = false;
  H = false;
  P1 = false;
  P2 = false;
  STOP = false;

  cmd.trim();
  if (cmd == "S") {
    STOP = true;
    return;
  }

  // Split by comma
  int start = 0;
  int end = cmd.indexOf(',');
  while (end != -1) {
    String part = cmd.substring(start, end);
    if (part == "F") W = true;
    else if (part == "B") S = true;
    else if (part == "L") A = true;
    else if (part == "R") D = true;
    else if (part == "J") J = true;
    else if (part == "K") K = true;
    else if (part == "P") P = true;
    else if (part == "H") H = true;
    else if (part == "P1") P1 = true;
    else if (part == "P2") P2 = true;
    start = end + 1;
    end = cmd.indexOf(',', start);
  }

  // Last part
  String part = cmd.substring(start);
  if (part == "F") W = true;
  else if (part == "B") S = true;
  else if (part == "L") A = true;
  else if (part == "R") D = true;
  else if (part == "J") J = true;
  else if (part == "K") K = true;
  else if (part == "P") P = true;
  else if (part == "H") H = true;
  else if (part == "P1") P1 = true;
  else if (part == "P2") P2 = true;
}

void CommunicationESP() {
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

  String receivedCmd = "";
  while (Wire.available()) {
    char c = Wire.read();
    if (c == '\n') break;
    receivedCmd += c;
  }

  Serial.print("CMD dari ESP: ");
  Serial.println(receivedCmd);

  parseCommand(receivedCmd);
}