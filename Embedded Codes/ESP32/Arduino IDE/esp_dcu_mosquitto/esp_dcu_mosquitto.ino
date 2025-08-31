#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// --- Wi-Fi Credentials ---
const char* ssid = "Delta_Bsnl-2.4G";
const char* password = "4712762392";

// --- MQTT Broker Details ---
const char* mqtt_broker = "192.168.1.48"; // Your Home Assistant's actual IP
const int mqtt_port = 1883;
const char* mqtt_username = "mqtt-user";     // From your Home Assistant MQTT settings
const char* mqtt_password = "012345"; // Your actual password for mqtt-user
const char *topic_publish = "esp_dcu";
// Digital Output Pins (4 pins for 4-bit output)
const int DIGIT_OUT_PINS[] = {21, 47, 48, 38};  // GPIO pins for digital output

// Digital Input Pins (4 pins for 4-bit input)
const int DIGIT_IN_PINS[] = {18, 17, 15, 7};  // GPIO pins for digital input

// Analog Input Pins
const int ANALOG_PIN_1 = 16;   // GPIO16 (ADC2_CH5) on ESP32-S3
const int ANALOG_PIN_2 = 5;    // GPIO5 (ADC1_CH4) on ESP32-S3

// Publishing interval (milliseconds)
const unsigned long PUBLISH_INTERVAL = 5000;  // Publish every 5 seconds
unsigned long lastPublishTime = 0;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
    Serial.begin(115200);
    
    Serial.println("ESP32 MQTT Sensor Publisher Starting...");
    
    // Configure ADC resolution to 12-bit (0-4095)
    analogReadResolution(12);
    
    // Configure ADC attenuation for full 0-3.3V range
    analogSetAttenuation(ADC_11db);
    
    // Initialize Digital Output Pins
    for (int i = 0; i < 4; i++) {
        pinMode(DIGIT_OUT_PINS[i], OUTPUT);
        digitalWrite(DIGIT_OUT_PINS[i], LOW);
    }
    
    // Initialize Digital Input Pins with internal pull-up
    for (int i = 0; i < 4; i++) {
        pinMode(DIGIT_IN_PINS[i], INPUT_PULLUP);
    }
    
    // Initialize Analog Pins (no special setup needed for ADC pins)
    pinMode(ANALOG_PIN_1, INPUT);
    pinMode(ANALOG_PIN_2, INPUT);
    
    // Set MQTT buffer size
    client.setBufferSize(1024);
    
    // WiFi connection
    connectWiFi();
    
    // MQTT connection
    client.setServer(mqtt_broker, mqtt_port);
    reconnectMQTT();
    
    // Seed random number generator
    randomSeed(analogRead(0));
    
    Serial.println("System Ready");
}

void connectWiFi() {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.println("WiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void loop() {
    // Maintain MQTT connection
    if (!client.connected()) {
        reconnectMQTT();
    }
    client.loop();
    
    // Check if WiFi is still connected
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected, reconnecting...");
        connectWiFi();
    }
    
    // Publish sensor data at regular intervals
    unsigned long currentTime = millis();
    if (currentTime - lastPublishTime >= PUBLISH_INTERVAL) {
        publishSensorData();
        lastPublishTime = currentTime;
    }
    
    delay(10);
}

void publishSensorData() {
    // Generate random 4-bit value for DIGIT_OUT (0x0 to 0xF)
    uint8_t digitOutValue = random(0, 16);  // 0 to 15 (0x0 to 0xF)
    
    // Write the 4-bit value to digital output pins
    setDigitalOutputs(digitOutValue);
    
    // Read 4-bit value from digital input pins
    uint8_t digitInValue = readDigitalInputs();
    
    // Read analog values (12-bit ADC: 0-4095)
    int analogValue1 = analogRead(ANALOG_PIN_1);
    int analogValue2 = analogRead(ANALOG_PIN_2);
    
    // Create JSON document
    DynamicJsonDocument doc(512);
    
    // Add sensor data to JSON
    char hexBuffer[8];
    
    // DIGIT_OUT as hex string
    sprintf(hexBuffer, "0x%X", digitOutValue);
    doc["ESP_DOUT"] = hexBuffer;
    
    // DIGIT_IN as hex string
    sprintf(hexBuffer, "0x%X", digitInValue);
    doc["ESP_DIN"] = hexBuffer;
    
    // Analog values
    doc["ESP_ADC1"] = analogValue1;
    doc["ESP_ADC2"] = analogValue2;
    
    // Serialize JSON to string
    String jsonString;
    serializeJson(doc, jsonString);
    
    // Publish to MQTT
    if (client.publish(topic_publish, jsonString.c_str())) {
        Serial.println("Published sensor data:");
        Serial.println(jsonString);
        
        // Print detailed info
        Serial.println("Details:");
        Serial.print("  DIGIT_OUT: 0x");
        Serial.print(digitOutValue, HEX);
        Serial.print(" (Binary: ");
        for (int i = 3; i >= 0; i--) {
            Serial.print((digitOutValue >> i) & 1);
        }
        Serial.println(")");
        
        Serial.print("  DIGIT_IN: 0x");
        Serial.print(digitInValue, HEX);
        Serial.print(" (Binary: ");
        for (int i = 3; i >= 0; i--) {
            Serial.print((digitInValue >> i) & 1);
        }
        Serial.println(")");
        
        Serial.print("  ANALOG_1: ");
        Serial.println(analogValue1);
        
        Serial.print("  ANALOG_2: ");
        Serial.println(analogValue2);
        Serial.println();
    } else {
        Serial.println("Failed to publish sensor data!");
    }
}

void setDigitalOutputs(uint8_t value) {
    // Set each bit of the 4-bit value to corresponding output pin
    for (int i = 0; i < 4; i++) {
        bool bitValue = (value >> i) & 1;
        digitalWrite(DIGIT_OUT_PINS[i], bitValue ? HIGH : LOW);
    }
}

uint8_t readDigitalInputs() {
    uint8_t value = 0;
    
    // Read each input pin and combine into 4-bit value
    for (int i = 0; i < 4; i++) {
        // Note: INPUT_PULLUP means LOW = pressed/connected, HIGH = released/open
        // Invert the reading if you want HIGH = 1, LOW = 0
        bool bitValue = !digitalRead(DIGIT_IN_PINS[i]);  // Inverted for pull-up
        if (bitValue) {
            value |= (1 << i);
        }
    }
    
    return value;
}

void reconnectMQTT() {
    while (!client.connected()) {
        String client_id = "esp32-sensor-";
        client_id += String(WiFi.macAddress());
        
        Serial.print("Connecting to MQTT broker...");
        
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println(" Connected!");
            Serial.print("Client ID: ");
            Serial.println(client_id);
        } else {
            Serial.print(" Failed, rc=");
            Serial.print(client.state());
            Serial.println(" Retrying in 5 seconds...");
            delay(5000);
        }
    }
}