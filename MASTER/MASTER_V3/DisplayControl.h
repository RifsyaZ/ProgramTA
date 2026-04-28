void parseCommand(String cmd) {
  // Reset semua flag
  isForward = false;
  isBackward = false;
  isLeft = false;
  isRight = false;
  isJump = false;
  isKill = false;
  isStop = false;

  if (cmd == "S") {
    isStop = true;
    return;
  }

  // Split by comma
  int start = 0;
  int end = cmd.indexOf(',');
  while (end != -1) {
    String part = cmd.substring(start, end);
    if (part == "F") isForward = true;
    else if (part == "B") isBackward = true;
    else if (part == "L") isLeft = true;
    else if (part == "R") isRight = true;
    else if (part == "J") isJump = true;
    else if (part == "K") isKill = true;
    start = end + 1;
    end = cmd.indexOf(',', start);
  }

  // Last part
  String part = cmd.substring(start);
  if (part == "F") isForward = true;
  else if (part == "B") isBackward = true;
  else if (part == "L") isLeft = true;
  else if (part == "R") isRight = true;
  else if (part == "J") isJump = true;
  else if (part == "K") isKill = true;
}

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