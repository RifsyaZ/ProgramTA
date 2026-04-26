void gyroCheck(int sp, int x) {
  MPU1();
  if (yaw > x + 2) {
    turnn(sp, -10);
    if (yaw >= x - 2 && yaw <= x + 2) {
      sv = 1;
      wheel(0, 0, 0);
    }
  } else if (yaw < x - 2) {
    turnn(sp, 10);
    if (yaw >= x - 2 && yaw <= x + 2) {
      sv = 1;
      wheel(0, 0, 0);
    }
  } else if (yaw >= x - 2 && yaw <= x + 2) {
    sv = 1;
    wheel(0, 0, 0);
  }
}

void ultraCheck(int sp, int s, int x) {
  ultrasonic();
  if (dis_kidep > x + 2) {
    wheel(sp, 0, s);
    if (dis_kidep >= x - 2 && dis_kidep <= x + 2) {
      sv = 2;
      wheel(0, 0, 0);
    }
  } else if (dis_kidep < x - 2) {
    wheel(sp, 0, -s);
    if (dis_kidep >= x - 2 && dis_kidep <= x + 2) {
      sv = 2;
      wheel(0, 0, 0);
    }
  } else if (dis_kidep >= x - 2 && dis_kidep <= x + 2) {
    sv = 2;
    wheel(0, 0, 0);
  }
}

void camBallCheck(int sp) {
  readCam_ball();
  if (ball == 1 || ball == 10) {
    turnn(sp, -10);
    if (ball == 100) {
      sv = 3;
      wheel(0, 0, 0);
    }
  } else if (ball == 2 || ball == 20) {
    turnn(sp, 10);
    if (ball == 100) {
      sv = 3;
      wheel(0, 0, 0);
    }
  } else if (ball == 100) {
    sv = 3;
    wheel(0, 0, 0);
  }
}

void camSiloCheck(int sp) {
  readCam_silo();
  if (silo == 1 || silo == 10) {
    turnn(sp, -10);
    if (silo == 100) {
      sv = 3;
      wheel(0, 0, 0);
    }
  } else if (silo == 2 || silo == 20) {
    turnn(sp, 10);
    if (silo == 100) {
      sv = 3;
      wheel(0, 0, 0);
    }
  } else if (silo == 100) {
    sv = 3;
    wheel(0, 0, 0);
  }
}
