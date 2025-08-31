#include <Wire.h>

// Define I2C pins
#define I2C_SDA PB11
#define I2C_SCL PB10

// Define control pin
#define CONTROL_PIN PB15

// Example I2C device address
#define DEVICE_ADDRESS 0x60

HardwareSerial MySerial1(PD6, PD5); // RX, TX for UART Debug

// Function declarations
void scanI2CDevices();
void ChannelTest(byte frame1, byte frame2, byte frame3);

void setup() {
  MySerial1.begin(115200);
  
  // Initialize control pin as output and set it LOW
  pinMode(CONTROL_PIN, OUTPUT);
  digitalWrite(CONTROL_PIN, LOW);
  
  // Initialize I2C with custom pins
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  
  // Set I2C clock speed (optional)
  Wire.setClock(100000); // 100kHz standard mode
  
  MySerial1.println("I2C initialized on PB10(SCL) and PB11(SDA)");
  MySerial1.println("PB15 set to LOW");

}

void loop() {
  // Change value according to data sheet to set value to 1 channel
 ChannelTest(0x5A,0x08,0x31);
}


// Send 3 8-bit frames in one transmission
  void ChannelTest(byte frame1, byte frame2, byte frame3){
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(frame1);  // First 8-bit frame
  Wire.write(frame2);  // Second 8-bit frame
  Wire.write(frame3);  // Third 8-bit frame
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    MySerial1.print("3 frames sent: 0x");
    MySerial1.print(frame1, HEX);
    MySerial1.print(" 0x");
    MySerial1.print(frame2, HEX);
    MySerial1.print(" 0x");
    MySerial1.println(frame3, HEX);
  } else {
    MySerial1.print("Frame transmission error: ");
    MySerial1.println(error);
  }
}
