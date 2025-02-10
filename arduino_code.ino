#include <WiFi.h>
#include <PubSubClient.h>

// WiFi credentials
const char* ssid = "sumit_vivo";
const char* password = "sumit123456";

// ThingsBoard server and device access token
const char* mqtt_server = "demo.thingsboard.io";
const char* deviceToken = "h9mxd9gmi67v1jvp8hn9"; 

void mqttCallback(char* topic, byte* payload, unsigned int length);

WiFiClient espClient;
PubSubClient client(espClient);

#define RX_PIN 16  // UART RX pin
#define TX_PIN 17  // UART TX pin

// Data variables
uint8_t forward_dist_1, forward_dist_2, backward_dist_1, backward_dist_2;
uint8_t motor1_speed1, motor1_speed2, motor2_speed1, motor2_speed2;
uint16_t forward_dist, backward_dist, motor1_speed, motor2_speed;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");

  // Initialize MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);

  delay(2000);
}

void loop() {
  checkWiFi();
  checkMQTT();

  if (Serial2.available() >= 8) {
    // Read 8 bytes from STM32
    forward_dist_1 = Serial2.read();
    forward_dist_2 = Serial2.read();
    motor1_speed1 = Serial2.read();
    motor1_speed2 = Serial2.read();
    backward_dist_1 = Serial2.read();
    backward_dist_2 = Serial2.read();
    motor2_speed1 = Serial2.read();
    motor2_speed2 = Serial2.read();

    // Combine bytes into 16-bit values
    forward_dist = ((forward_dist_1 << 8) | (forward_dist_2));
    backward_dist = ((backward_dist_1 << 8) | (backward_dist_2));
    motor1_speed = ((motor1_speed1 << 8) | (motor1_speed2));
    motor2_speed = ((motor2_speed1 << 8) | (motor2_speed2));

    // Print received data
    Serial.printf("Forward Distance: %d cm\n", forward_dist);
    Serial.printf("Backward Distance: %d cm\n", backward_dist);
    Serial.printf("Motor1 Speed: %d RPM\n", motor1_speed);
    Serial.printf("Motor2 Speed: %d RPM\n", motor2_speed);

    // Create JSON payload
    String payload = "{\"forward_dist\": " + String(forward_dist) + 
                     ", \"backward_dist\": " + String(backward_dist) + 
                     ", \"motor1_speed\": " + String(motor1_speed) + 
                     ", \"motor2_speed\": " + String(motor2_speed) + "}";

    // Publish to ThingsBoard
    if (client.publish("v1/devices/me/telemetry", payload.c_str())) {
        Serial.println("Data sent to ThingsBoard!");
    } else {
        Serial.println("Failed to send data to ThingsBoard.");
    }

    delay(2000);
  }

  client.loop();
}

// Ensure WiFi is connected
void checkWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }
    Serial.println("\nReconnected to WiFi!");
  }
}

// Ensure MQTT is connected
void checkMQTT() {
  if (!client.connected()) {
    Serial.print("Reconnecting to MQTT...");
    while (!client.connected()) {
      if (client.connect("ESP32_1", deviceToken, "")) {
        Serial.println("Connected!");
      } else {
        Serial.print("Failed, rc=");
        Serial.println(client.state());
        delay(5000);
      }
    }
  }
}

// MQTT callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Handle incoming messages (if needed)
}
