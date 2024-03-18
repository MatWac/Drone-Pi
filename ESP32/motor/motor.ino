#define RX_PIN 16  // Pin de réception
#define TX_PIN 17  // Pin d'émission

HardwareSerial MySerial(2);  // Utilisation de UART2

void setup() {
  Serial.begin(115200);
  MySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // Configuration de UART2
}

void loop() {
  if (MySerial.available()) {
    String received = MySerial.readStringUntil('\n');
    Serial.println("Reçu : " + received);
  }
}
