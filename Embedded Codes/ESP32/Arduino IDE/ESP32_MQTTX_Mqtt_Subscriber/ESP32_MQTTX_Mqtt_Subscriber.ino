#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> 

// WiFi
const char *ssid = "your_Wi-Fi_ssid"; // Enter your Wi-Fi name
const char *password = "your_Wi-Fi_passward";  // Enter Wi-Fi password

// MQTT Broker
const char *mqtt_broker = "broker.emqx.io";
const char *topic = "Your_topic";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
    // Set serial baud to 115200;
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
    // Convert payload (byte array) to a String
  String receivedJsonString = "";
  for (int i = 0; i < length; i++) {
    receivedJsonString += (char)payload[i];
  }
  Serial.print("Payload (JSON String): ");
  Serial.println(receivedJsonString);
  Serial.println("----------------------------------");

  // --- Send the entire received JSON string to Serial Monitor ---
  Serial.print(receivedJsonString);
  Serial.println("JSON data sent to external device via UART1."); // Prints to Serial Monitor (UART0)
}

void loop() {
    client.loop();
}
