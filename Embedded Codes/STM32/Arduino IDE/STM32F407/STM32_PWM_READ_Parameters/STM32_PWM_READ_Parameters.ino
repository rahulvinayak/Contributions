#include <Arduino.h>

// Pin definition
#define PWM_INPUT_PIN PA1  // Must be a timer-capable pin


// Measurement variables
volatile unsigned long rising_time = 0;
volatile unsigned long falling_time = 0;
volatile unsigned long last_rising_time = 0;
volatile unsigned long period_us = 0;
volatile unsigned long pulse_width_us = 0;
volatile bool new_data = false;
volatile bool first_edge = true;

// Results
float frequency = 0;
float duty_cycle = 0;

void setup() {
  Serial.begin(115200);
  delay(2000); // Give time for serial to initialize
  
  Serial.println("STM32 PWM Reader");
  Serial.println("=========================================");
  
  // Configure input pin
  pinMode(PWM_INPUT_PIN, INPUT);
  
  // Attach interrupt for both edges
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN), pwm_interrupt, CHANGE);
  
  Serial.println("Setup complete!");
  Serial.print("Reading PWM on pin: ");
  Serial.println(PWM_INPUT_PIN);
  Serial.println("Connect your PWM signal and wait for measurements...");
  Serial.println("----------------------------------------");
}

void loop() {
  // Check for new data
  if (new_data) {
    calculateResults();
    displayResults();
    delay(1000);
    new_data = false;
  }
  
  // Check if no signal detected
  static unsigned long last_display = 0;
  if (millis() - last_display > 3000) {
    if (frequency == 0) {
      Serial.println("No PWM signal detected. Check connections.");
    }
    last_display = millis();
  }
  
  delay(100);
}

void pwm_interrupt() {
  unsigned long current_time = micros();
  
  if (digitalRead(PWM_INPUT_PIN) == HIGH) {
    // Rising edge detected
    rising_time = current_time;
    
    // Calculate period (time between rising edges)
    if (!first_edge && last_rising_time != 0) {
      period_us = current_time - last_rising_time;
    }
    
    last_rising_time = current_time;
    first_edge = false;
    
  } else {
    // Falling edge detected
    falling_time = current_time;
    
    // Calculate pulse width
    if (rising_time != 0 && falling_time > rising_time) {
      pulse_width_us = falling_time - rising_time;
      new_data = true;
    }
  }
}

void calculateResults() {
  if (period_us > 0 && pulse_width_us > 0) {
    // Calculate frequency
    frequency = 1000000.0 / (float)period_us;
    
    // Calculate duty cycle
    duty_cycle = ((float)pulse_width_us / (float)period_us) * 100.0;
  }
}

void displayResults() {
  Serial.println("=== PWM Measurement Results ===");
  
  Serial.print("Frequency: ");
  if (frequency >= 1000) {
    Serial.print(frequency / 1000.0, 2);
    Serial.println(" kHz");
  } else {
    Serial.print(frequency, 2);
    Serial.println(" Hz");
  }
  
  Serial.print("Period: ");
  Serial.print(period_us);
  Serial.println(" us");
  
  Serial.print("Pulse Width: ");
  Serial.print(pulse_width_us);
  Serial.println(" us");
  
  Serial.print("Duty Cycle: ");
  Serial.print(duty_cycle, 1);
  Serial.println(" %");
  
  Serial.println("-------------------------------");
}
