#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>

// --- Wi-Fi Credentials ---
const char* ssid = "Delta_Bsnl-2.4G";
const char* password = "4712762392";

const char* mqtt_broker = "28d9815c3c984006af9c06844377251e.s1.eu.hivemq.cloud";  // e.g., "a1b2c3d4.s1.eu.hivemq.cloud"
const int mqtt_port = 8883;  // TLS port
const char* mqtt_username = "webyfyiot";
const char* mqtt_password = "Webyfyiot@123";
const char *topic_publish = "esp_dcu";

// --- PWM Configuration ---
const int PWM_PIN = 12;           // GPIO pin for PWM output
const int PWM_FREQUENCY = 5000;   // 5kHz frequency
const int PWM_RESOLUTION = 12;    // 12-bit resolution (0-4095)

// Digital Output Pins (4 pins for 4-bit output)
const int DIGIT_OUT_PINS[] = {21, 47, 48, 38};  // GPIO pins for digital output

// Digital Input Pins (4 pins for 4-bit input)
const int DIGIT_IN_PINS[] = {18, 17, 15, 7};  // GPIO pins for digital input

// Analog Input Pins
const int ANALOG_PIN_1 = 16;   // GPIO16 (ADC2_CH5) on ESP32-S3 - Used for PWM control
const int ANALOG_PIN_2 = 5;    // GPIO5 (ADC1_CH4) on ESP32-S3

// Publishing interval (milliseconds)
const unsigned long PUBLISH_INTERVAL = 5000;  // Publish every 5 seconds
unsigned long lastPublishTime = 0;

// --- NTP Configuration for TLS ---
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;  // GMT+5:30 for India
const int daylightOffset_sec = 0;

// --- HiveMQ Cloud Root CA Certificate ---
// ISRG Root X1 certificate used by Let's Encrypt (common for HiveMQ Cloud)
const char* root_ca = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

WiFiClientSecure espClient;
PubSubClient client(espClient);

void setup() {
    Serial.begin(115200);
    
    Serial.println("ESP32 MQTT Sensor Publisher with PWM Starting...");
    
    // Configure ADC resolution to 12-bit (0-4095)
    analogReadResolution(12);
    
    // Configure ADC attenuation for full 0-3.3V range
    analogSetAttenuation(ADC_11db);
    
    // Initialize PWM
    setupPWM();
    
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
    
    // Sync time for TLS certificate validation
    syncTime();
    
    // Configure TLS/SSL
    espClient.setCACert(root_ca);
    
    // Optional: Skip server certificate verification for testing (NOT recommended for production)
    // espClient.setInsecure();
    
    // MQTT connection
    client.setServer(mqtt_broker, mqtt_port);
    reconnectMQTT();
    
    // Seed random number generator
    randomSeed(analogRead(0));
    
    Serial.println("System Ready");
}

void setupPWM() {
    // Configure PWM using new API (ESP32 Arduino Core 3.x)
    ledcAttach(PWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
    
    // Start with 0% duty cycle
    ledcWrite(PWM_PIN, 0);
    
    Serial.println("PWM initialized:");
    Serial.print("  Pin: GPIO");
    Serial.println(PWM_PIN);
    Serial.print("  Frequency: ");
    Serial.print(PWM_FREQUENCY);
    Serial.println(" Hz");
    Serial.print("  Resolution: ");
    Serial.print(PWM_RESOLUTION);
    Serial.println(" bits");
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

void syncTime() {
    Serial.print("Syncing time for TLS");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    
    time_t now = time(nullptr);
    while (now < 8 * 3600 * 2) {
        delay(500);
        Serial.print(".");
        now = time(nullptr);
    }
    Serial.println(" Done!");
    
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    Serial.print("Current time: ");
    Serial.print(asctime(&timeinfo));
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
    

    // Update PWM duty cycle based on analogValue1
    int pwmDuty = analogValue1;  // Direct mapping since both are 0-4095
    ledcWrite(PWM_PIN, pwmDuty);
    
    // Calculate duty cycle percentage
    float dutyPercent = (float)pwmDuty / 4095.0 * 100.0;
    
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
    
    // PWM data - combine frequency and duty in one key
    String pwmData = String(PWM_FREQUENCY) + "," + String(dutyPercent, 2);
    doc["ESP_PWM"] = pwmData;
    
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
        
        Serial.print("  PWM: ");
        Serial.print(PWM_FREQUENCY);
        Serial.print(" Hz, ");
        Serial.print(dutyPercent, 2);
        Serial.print("% duty (raw: ");
        Serial.print(pwmDuty);
        Serial.println(")");
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