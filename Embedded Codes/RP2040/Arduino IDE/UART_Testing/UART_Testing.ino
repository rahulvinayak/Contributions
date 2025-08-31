//Uart Testing

void setup() {
  Serial.begin(115200); //UART Initialization
}

void loop() {
  Serial.println("Hello User"); //Prints "Hello User" in serial monitor
  delay(1000);
}
