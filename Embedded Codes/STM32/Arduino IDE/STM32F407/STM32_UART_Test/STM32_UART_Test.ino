HardwareSerial MySerial1(PD6, PD5); // RX, TX

void setup() {
  MySerial1.begin(115200); // Set Serial Monitor Baud Rate to 115200
}

void loop() {
 MySerial1.println("UART Testing"); // Sends Message at Inverval of 1 sec
 delay(1000);
}
