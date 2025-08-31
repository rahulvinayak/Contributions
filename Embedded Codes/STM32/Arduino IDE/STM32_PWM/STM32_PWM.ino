#include <Arduino.h>

// Pin definition
#define PWM_INPUT_PIN PA1  // Must be a timer-capable pin
HardwareSerial MySerial1(PD6, PD5); // RX, TX


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
  MySerial1.begin(115200);
  delay(2000); // Give time for serial to initialize
  
  MySerial1.println("STM32 PWM Reader");
  MySerial1.println("=========================================");
  
  // Configure input pin
  pinMode(PWM_INPUT_PIN, INPUT);
  
  // Attach interrupt for both edges
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN), pwm_interrupt, CHANGE);
  
  MySerial1.println("Setup complete!");
  MySerial1.print("Reading PWM on pin: ");
  MySerial1.println(PWM_INPUT_PIN);
  MySerial1.println("Connect your PWM signal and wait for measurements...");
  MySerial1.println("----------------------------------------");
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
      MySerial1.println("No PWM signal detected. Check connections.");
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
  MySerial1.println("=== PWM Measurement Results ===");
  
  MySerial1.print("Frequency: ");
  if (frequency >= 1000) {
    MySerial1.print(frequency / 1000.0, 2);
    MySerial1.println(" kHz");
  } else {
    MySerial1.print(frequency, 2);
    MySerial1.println(" Hz");
  }
  
  MySerial1.print("Period: ");
  MySerial1.print(period_us);
  MySerial1.println(" us");
  
  MySerial1.print("Pulse Width: ");
  MySerial1.print(pulse_width_us);
  MySerial1.println(" us");
  
  MySerial1.print("Duty Cycle: ");
  MySerial1.print(duty_cycle, 1);
  MySerial1.println(" %");
  
  MySerial1.println("-------------------------------");
}
