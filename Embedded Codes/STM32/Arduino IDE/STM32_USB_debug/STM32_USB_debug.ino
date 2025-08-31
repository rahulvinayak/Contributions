void setup() {
  // Start USB Serial
  Serial.begin(115200);  // works over USB
  while (!Serial) {
    ; // wait for USB Serial to be recognized
  }

  Serial.println("USB Serial Debug Initialized");
}

void loop() {
  static int counter = 0;
  Serial.print("Debug count: ");
  Serial.println(counter++);
  delay(1000);
}
