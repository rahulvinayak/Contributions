#include <Arduino.h>

String receivedData = "";
String expectedMessage = "hii esp";
String responseMessage = "hello stm\r\n";

void setup() {
  Serial.setPins(20,21);
  Serial.begin(115200);
  Serial.setTimeout(1000); // Set timeout for Serial operations
  
  
  Serial.println("ESP ready - waiting for STM32...");
}

void loop() {
  // Check if data is available on UART
  if (Serial.available()) {
    // Read the incoming data
    receivedData = Serial.readString();
    receivedData.trim(); // Remove any trailing whitespace/newlines
    
    // Print received data for debugging
    Serial.print("Received: ");
    Serial.println(receivedData);
    
    // Check if received message matches expected
    if (receivedData.indexOf(expectedMessage) >= 0) {
      // Send confirmation response to STM32
      Serial.print(responseMessage);
      
      
      Serial.println("Response sent to STM32");
    }
    else {
      Serial.println("Unexpected message from STM32");
    }
    
    // Clear the received data buffer
    receivedData = "";
  }
  
  delay(10); // Small delay to prevent overwhelming the serial buffer
}

// Alternative implementation using interrupt-based approach
void setup_interrupt() {
  Serial.begin(115200);

  
  // Enable serial event interrupt (if supported by your ESP variant)
  Serial.println("ESP ready with interrupt handling");
}

void loop_interrupt() {
  // Main loop can do other tasks
  delay(100);
}

// Serial event handler (called automatically when data arrives)
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    receivedData += inChar;
    
    // Check for end of message (newline)
    if (inChar == '\n') {
      receivedData.trim();
      
      if (receivedData.indexOf(expectedMessage) >= 0) {
        Serial.print(responseMessage);

      }
      
      receivedData = ""; // Clear buffer
    }
  }
}