#include <SPI.h>

#define SS_PIN 5

void setup() {
  Serial.begin(115200);

  SPI.begin(18, 19, 23, SS_PIN);  // SCK, MISO, MOSI, SS
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
}

void loop() {
  const char* message = "Hello STM32";
  digitalWrite(SS_PIN, LOW);
  for (size_t i = 0; i < strlen(message); i++) {
    SPI.transfer(message[i]);
  }
  digitalWrite(SS_PIN, HIGH);

  Serial.println("Message sent to STM32");
  delay(3000);
}
