#include <SPI.h>
#include <Ethernet.h>

// MAC address for your controller (must be unique on your network)
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// IP address configuration for direct connection (no router)
IPAddress ip(192, 168, 1, 10);       // Static IP for Arduino
IPAddress gateway(192, 168, 1, 1);   // Not used in direct connection
IPAddress subnet(255, 255, 255, 0);  // Subnet mask

// Create server on port 80
EthernetServer server(80);

// Global client to maintain connection
EthernetClient globalClient;
unsigned long lastDataSent = 0;
unsigned long messageCounter = 0;
bool clientConnected = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println("W5500 Direct Ethernet Connection - Continuous Hello Server");
  Serial.println("Configure your PC IP to 192.168.1.11 for direct connection");
  
  // Initialize Ethernet with W5500
  // The W5500 uses SPI pins: MOSI(11), MISO(12), SCK(13), CS(10)
  Ethernet.begin(mac, ip, gateway, subnet);
  
  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found. Check wiring.");
    while (true) {
      delay(1); // Do nothing forever
    }
  }
  
  // Check for Ethernet cable
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
  
  // Start the server
  server.begin();
  Serial.print("Server started at IP: ");
  Serial.println(Ethernet.localIP());
  Serial.println("Direct connection ready - Connect via Ethernet cable");
  Serial.println("Use Telnet or TCP client: telnet 192.168.1.10 80");
  Serial.println("Connection will stay open and send continuous data");
  Serial.println("----------------------------------------");
}

void loop() {
  // Check for new clients if no client is connected
  if (!clientConnected) {
    EthernetClient newClient = server.available();
    if (newClient) {
      globalClient = newClient;
      clientConnected = true;
      messageCounter = 0;
      
      Serial.println("New client connected - Starting continuous data stream");
      Serial.println("Client IP: " + String(globalClient.remoteIP()[0]) + "." + 
                                    String(globalClient.remoteIP()[1]) + "." + 
                                    String(globalClient.remoteIP()[2]) + "." + 
                                    String(globalClient.remoteIP()[3]));
      Serial.println("----------------------------------------");
      
      // Send initial headers (for HTTP clients)
      globalClient.println("HTTP/1.1 200 OK");
      globalClient.println("Content-Type: text/plain");
      globalClient.println("Cache-Control: no-cache");
      globalClient.println("Connection: keep-alive");
      globalClient.println();
      globalClient.println("=== Arduino W5500 Continuous Data Stream ===");
      globalClient.println("Connection established. Sending data every 1 second...");
      globalClient.println();
      
      lastDataSent = millis();
    }
  }
  
  // If client is connected, send data continuously
  if (clientConnected && globalClient.connected()) {
    // Send data every 1 second
    if (millis() - lastDataSent >= 1000) {
      messageCounter++;
      
      // Create the message
      String message = "Hello #" + String(messageCounter) + 
                      " | Time: " + String(millis()) + "ms" +
                      " | Uptime: " + String(millis()/1000) + "s";
      
      // Send to client
      globalClient.println(message);
      globalClient.flush(); // Ensure data is sent immediately
      
      // Display on serial monitor
      Serial.println("Sent: " + message);
      
      lastDataSent = millis();
    }
    
    // Check if client is still connected
    if (!globalClient.connected()) {
      Serial.println("Client disconnected after " + String(messageCounter) + " messages");
      Serial.println("----------------------------------------");
      clientConnected = false;
      globalClient.stop();
    }
  }
  
  // Send periodic status to serial monitor
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 10000) { // Every 10 seconds
    if (clientConnected) {
      Serial.println("Status: Client connected, " + String(messageCounter) + " messages sent");
    } else {
      Serial.println("Status: Waiting for client connection...");
      Serial.println("Server running at: " + String(Ethernet.localIP()[0]) + "." + 
                                           String(Ethernet.localIP()[1]) + "." + 
                                           String(Ethernet.localIP()[2]) + "." + 
                                           String(Ethernet.localIP()[3]));
    }
    lastStatus = millis();
  }
  
  // Small delay to prevent overwhelming the system
  delay(10);
}