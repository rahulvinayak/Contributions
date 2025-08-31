#define TestPin PE13 // Change pins from PE8 to PE15 to Test

void setup() {
  pinMode(TestPin, OUTPUT);  // Set pin as output
}

void loop() {
  //Blink an LED attached to the Digital Pin to be Tested
  digitalWrite(TestPin, HIGH); // Turn LED on
  delay(500);                 // Wait 500ms
  digitalWrite(TestPin, LOW);  // Turn LED off
  delay(500);                 // Wait 500ms
  
}
