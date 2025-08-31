/*
 * STM32 SPI Slave - Receive Data from ESP32
 * 
 * Hardware Setup:
 * ESP32 -> STM32
 * MOSI (GPIO23) -> MISO (PB4)
 * SCK (GPIO18)  -> SCK (PB3)
 * CS (GPIO5)    -> NSS (PB6) [optional]
 * GND           -> GND
 * 
 * This code receives data from ESP32 SPI master
 */

#include <SPI.h>

String receivedMessage = "";
bool messageComplete = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("STM32 SPI Slave - Receiving from ESP32");
  Serial.println("Waiting for ESP32 data...");
  Serial.println();
  
  // Configure SPI pins for slave mode
  SPI.setMOSI(PB5);  // Not used but needed for init
  SPI.setMISO(PB4);  // This is our receive pin (connects to ESP32 MOSI)
  SPI.setSCLK(PB3);  // Clock from ESP32
  
  // Optional: Configure NSS/CS pin
  pinMode(PB6, INPUT);
  
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
}

void loop() {
  // Check if CS is active (low) - optional if using CS
  bool csActive = (digitalRead(PB6) == LOW);
  
  // Receive data when CS is active or continuously if not using CS
  if (csActive || true) {  // Change to just 'csActive' if using CS properly
    
    // Receive byte from ESP32
    uint8_t receivedByte = SPI.transfer(0x00);  // Send dummy, receive actual data
    
    if (receivedByte != 0x00) {  // Valid data received
      
      // Add to message string
      receivedMessage += (char)receivedByte;
      
      // Display received byte
      Serial.print("Received byte: 0x");
      Serial.print(receivedByte, HEX);
      Serial.print(" ('");
      Serial.print((char)receivedByte);
      Serial.println("')");
      
      // Check if message is complete (you can modify this logic)
      if (receivedByte == '!' || receivedMessage.length() >= 20) {
        messageComplete = true;
      }
    }
  }
  
  // If CS goes high, consider message complete
  if (!csActive && receivedMessage.length() > 0) {
    messageComplete = true;
  }
  
  // Display complete message
  if (messageComplete && receivedMessage.length() > 0) {
    Serial.println();
    Serial.print("Complete message: \"");
    Serial.print(receivedMessage);
    Serial.println("\"");
    Serial.println();
    
    // Reset for next message
    receivedMessage = "";
    messageComplete = false;
  }
  
  delay(10);  // Small delay
}