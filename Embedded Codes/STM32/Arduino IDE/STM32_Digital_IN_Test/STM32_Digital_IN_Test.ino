
#define buttonPin  PD8   // Input pin for button PD8 to PD15 for Test
#define ledPin  PB1      // Output pin for LED OnBoard

void setup() {
  // Initialize pin modes
  pinMode(buttonPin,INPUT_PULLDOWN); // Active-low button with internal pull-up
  pinMode(ledPin, OUTPUT);

  // Set initial LED state to OFF
  digitalWrite(ledPin, LOW);
}

void loop() {
  // Check if button is pressed (active low)
  if (digitalRead(buttonPin) == HIGH) {
    // Turn on LED
    digitalWrite(ledPin, HIGH);
  } else {
    // Turn off LED
    digitalWrite(ledPin, LOW);
  }

  // Debounce delay
  delay(100);
}
