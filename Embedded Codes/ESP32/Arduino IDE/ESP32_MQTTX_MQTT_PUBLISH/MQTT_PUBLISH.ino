#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> // Include the ArduinoJson library

long lastPublishTime = 0;
// WiFi
const char *ssid = "Your_Wifi_ssid"; // Enter your Wi-Fi name
const char *password = "Your_wifi_password";  // Enter Wi-Fi password

// MQTT Broker
const char *mqtt_broker = "broker.emqx.io";
const char *topic = "Your_Topic";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
    // Set software serial baud to 115200;
    Serial.begin(115200);
    // Connecting to a WiFi network
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the Wi-Fi network");
    //connecting to a mqtt broker
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);
    while (!client.connected()) {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Public EMQX MQTT broker connected");
        } else {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
    // Publish and subscribe
    client.subscribe(topic);
}

void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");
}

void loop() {
  long now = millis();
  if (now - lastPublishTime > 10000) { // Publish every 5 seconds (you can change this interval)
    lastPublishTime = now;

    StaticJsonDocument<100> doc; 
    doc["DIGIT_OUT"]= "255";
    doc["Message"]= "Hello From ESP32";     //Sample Message

    // Serialize JSON to a string
    String jsonString;
    serializeJson(doc, jsonString);

    Serial.print("Publishing ADC command: ");
    Serial.println(jsonString);

    // Publish the JSON string to the defined topic
    client.publish(topic, jsonString.c_str());
}
    client.loop();
}
