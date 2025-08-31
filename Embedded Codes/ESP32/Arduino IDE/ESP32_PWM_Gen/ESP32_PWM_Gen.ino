/*
 * ESP32S NodeMCU PWM Generation Example
 */


const int pwmPin = 12;    // GPIO4 - PWM pin

// PWM properties
const int freq = 5000;          // PWM frequency in Hz

const int pwmChannel2 = 1;      // PWM channel (0-15)
const int resolution = 8;       // PWM resolution (1-16 bits)

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 PWM Generation Started");
  
  // Configure PWM channels

  ledcAttach(pwmPin, freq, resolution);
  
  Serial.println("PWM channels configured successfully");
  Serial.println("Pin 12: Variable duty cycle, 5kHz");
}

void loop() {
  // Static PWM on pin 12 (50% duty cycle)
  ledcWrite(pwmPin, 127); // 127/255 = ~50% duty cycle
 
  
  Serial.println("PWM cycle completed");
  delay(1000);
}

/*
 * Alternative simple PWM using analogWrite() function
 * Note: This uses default PWM settings
 */
void simplePWMExample() {
  // Simple PWM generation (uncomment to use)
  // analogWrite(2, 127);  // 50% duty cycle on GPIO2
  // analogWrite(4, 64);   // 25% duty cycle on GPIO4
  // analogWrite(5, 191);  // 75% duty cycle on GPIO5
}
