#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// --- Wi-Fi Credentials ---
const char* ssid = "Your_Wifi_ssid";
const char* password = "Your_Wifi_Password";

// --- MQTT Broker Details ---
const char* mqtt_broker = "Your_Mqtt_Broker_IP"; // Your Mqtt Broker IP
const int mqtt_port = 1883;
const char* mqtt_username = "mqtt-user";     // From your Home Assistant MQTT settings
const char* mqtt_password = "012345"; // Your actual password for mqtt-user
const char *topic_publish = "Your_Publish_Topic";


// Publishing interval (milliseconds)
const unsigned long PUBLISH_INTERVAL = 5000;  // Publish every 5 seconds
unsigned long lastPublishTime = 0;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
    Serial.begin(115200);
    
    Serial.println("ESP32 MQTT Publisher Starting...");
    
    
    // Set MQTT buffer size
    client.setBufferSize(256);
    
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
        publishData();
        lastPublishTime = currentTime;
    }
    
    delay(10);
}

void publishData() {
    
    // Create JSON document
    DynamicJsonDocument doc(512);

    // Analog values
    doc["Message"] = string("Hello From ESP32");
    
    // Serialize JSON to string
    String jsonString;
    serializeJson(doc, jsonString);
    
    // Publish to MQTT
    if (client.publish(topic_publish, jsonString.c_str())) {
        Serial.println("Published sensor data:");
        Serial.println(jsonString);
    } else {
        Serial.println("Failed to publish sensor data!");
    }
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