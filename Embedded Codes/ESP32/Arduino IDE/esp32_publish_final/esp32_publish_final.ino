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
const char *topic_receive = "command";
const char *topic_response = "response";
#define UART1_RX_PIN 44  // Connect to STM32 Serial3 RX (PB6)
#define UART1_TX_PIN 43  // Connect to STM32 Serial3 TX (PB7)
#define UART1_BAUD_RATE 115200

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
    Serial.begin(9600);
    Serial2.begin(UART1_BAUD_RATE, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
    
    Serial.println("ESP32 MQTT-STM32 Bridge Starting...");
    
    // Set larger MQTT buffer size for large responses
    client.setBufferSize(4096);
    
    // WiFi connection
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected: " + WiFi.localIP().toString());
    
    // MQTT connection
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(mqttCallback);
    reconnectMQTT();
    
    Serial.println("System Ready");
    Serial.println("Waiting 5 seconds for STM32 I2C slave to initialize...");
    delay(5000); // Give STM32 time to fully boot and initialize I2C slave
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
    // Convert payload to string
    String receivedJsonString = "";
    for (int i = 0; i < length; i++) {
        receivedJsonString += (char)payload[i];
    }
    
    Serial.println("MQTT Command: " + receivedJsonString);
    
    // Forward to STM32
    forwardToSTM32(receivedJsonString);
}

void forwardToSTM32(String command) {
    // Clear any pending data first
    while (Serial2.available()) {
        Serial2.read();
    }
    
    // Send command with explicit newline
    Serial2.print(command);
    Serial2.print('\n');
    Serial2.flush();
}

void loop() {
    client.loop();
    
    // Handle STM32 responses
    handleSTM32Responses();
    
    // Reconnect if needed
    if (!client.connected()) {
        reconnectMQTT();
    }
    
    delay(10);
}

void handleSTM32Responses() {
    if (Serial2.available()) {
        String responses = readSTM32Responses();
        
        if (responses.length() > 0) {
            processMultipleResponses(responses);
        }
    }
}

String readSTM32Responses() {
    String allResponses = "";
    unsigned long startTime = millis();
    unsigned long lastDataTime = millis();
    int jsonCount = 0;
    
    // Read for up to 5 seconds
    while (millis() - startTime < 5000) {
        if (Serial2.available()) {
            String line = Serial2.readStringUntil('\n');
            line.trim();
            
            if (line.length() > 0) {
                lastDataTime = millis();
                
                // Check if line is valid JSON
                if (isValidJSON(line)) {
                    jsonCount++;
                    
                    if (allResponses.length() > 0) {
                        allResponses += "\n";
                    }
                    allResponses += line;
                }
                // Ignore non-JSON lines silently
                
                // Prevent memory overflow
                if (allResponses.length() > 8192) {
                    break;
                }
            }
        } else if (jsonCount > 0) {
            // If we have responses and no new data for 1 second, assume complete
            if (millis() - lastDataTime > 1000) {
                break;
            }
        }
        delay(10);
    }
    
    return allResponses;
}

bool isValidJSON(String jsonString) {
    if (jsonString.length() == 0) return false;
    if (!jsonString.startsWith("{") || !jsonString.endsWith("}")) return false;
    
    DynamicJsonDocument testDoc(2048);
    DeserializationError error = deserializeJson(testDoc, jsonString);
    return (error == DeserializationError::Ok);
}

void processMultipleResponses(String responses) {
    // Create a combined JSON object
    DynamicJsonDocument combinedDoc(4096);
    int responseCount = 0;
    int startPos = 0;
    
    while (startPos < responses.length()) {
        int endPos = responses.indexOf('\n', startPos);
        if (endPos == -1) endPos = responses.length();
        
        String singleResponse = responses.substring(startPos, endPos);
        singleResponse.trim();
        
        if (singleResponse.length() > 0 && isValidJSON(singleResponse)) {
            // Parse the individual response
            DynamicJsonDocument responseDoc(1024);
            DeserializationError error = deserializeJson(responseDoc, singleResponse);
            
            if (!error) {
                // Add each key-value pair to the combined response
                for (JsonPair kv : responseDoc.as<JsonObject>()) {
                    combinedDoc[kv.key()] = kv.value();
                }
                responseCount++;
            }
        }
        
        startPos = endPos + 1;
    }
    
    // Publish the combined response
    if (responseCount > 0) {
        String combinedResponse;
        serializeJson(combinedDoc, combinedResponse);
        publishSingleResponse(combinedResponse);
    }
}

void publishSingleResponse(String response) {
    // Validate JSON before publishing
    DynamicJsonDocument testDoc(4096);
    DeserializationError error = deserializeJson(testDoc, response);
    
    if (error) {
        // Create error response
        DynamicJsonDocument errorDoc(1024);
        errorDoc["status"] = "error";
        errorDoc["message"] = "Invalid JSON from STM32";
        errorDoc["error_code"] = error.c_str();
        errorDoc["timestamp"] = millis();
        
        String errorResponse;
        serializeJson(errorDoc, errorResponse);
        response = errorResponse;
    }
    
    // Publish to MQTT
    char responseBuffer[4096];
    int copyLength = min(response.length(), sizeof(responseBuffer) - 1);
    response.substring(0, copyLength).toCharArray(responseBuffer, sizeof(responseBuffer));
    
    if (client.publish(topic_response, responseBuffer)) {
        Serial.println("Response sent to MQTT (" + String(strlen(responseBuffer)) + " bytes)");
    } else {
        Serial.println("MQTT publish failed");
    }
}

void reconnectMQTT() {
    while (!client.connected()) {
        String client_id = "esp32-stm32-bridge-";
        client_id += String(WiFi.macAddress());
        
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            client.subscribe(topic_receive);
            Serial.println("MQTT Connected");
        } else {
            Serial.println("MQTT connection failed, retrying...");
            delay(5000);
        }
    }
}