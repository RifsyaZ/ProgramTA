#define Sensor_49E_kiri PA4
#define Sensor_49E_kanan PA5

volatile int Arah = 1;  // Default arah = 1

void setup() {
  pinMode(Sensor_49E_kiri, INPUT);
  pinMode(Sensor_49E_kanan, INPUT);
  Serial.begin(9600);
}

int ArahHoming() {
  int kiri = analogRead(Sensor_49E_kiri);
  int kanan = analogRead(Sensor_49E_kanan);
  
  if (kiri >= 950 && kanan <= 450) {
    Arah = 1;
  } else if (kiri >= 950 && kanan >= 950) {
    Arah = 2;
  } else if (kiri <= 450 && kanan <= 450) {
    Arah = 3;
  }
  // Selain itu, Arah tidak berubah (tetap nilai sebelumnya)
  
  return Arah;
}

void loop() {
  Serial.print("Kiri: ");
  Serial.print(analogRead(Sensor_49E_kiri));
  Serial.print(" | Kanan: ");
  Serial.print(analogRead(Sensor_49E_kanan));
  Serial.print(" | Arah: ");
  Serial.println(ArahHoming());
  delay(100);
}