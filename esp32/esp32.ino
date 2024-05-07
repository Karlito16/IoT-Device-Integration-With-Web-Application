#include <WiFi.h>
#include <PubSubClient.h>

// Including custom configuration
#include "configuration.h"

// Creating an instance of PubSubClient client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Trace motion sensor status
unsigned long motionDetected;

// Trace current time, used for ping
unsigned long currentTime;

void ulongToChar(unsigned long num, char* buffer) {
  sprintf(buffer, "%lu", num);
}

void receiveCallback(char* topic, byte* payload, unsigned int length) {
  Serial.println("=====Message received=====");
  Serial.printf("Topic: %s\n", topic);

  Serial.print("Payload: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void connectToMQTT(const char* clientId, const char* topic, int retryInterval, bool reconnecting) {
  while (!mqttClient.connected()) {
    Serial.printf("%s to MQTT ...\n", reconnecting ? "Reconnecting" : "Connecting");

    // Try to connect to MQTT
    if (mqttClient.connect(clientId)) {
      Serial.println("Connected to MQTT!");
    } else {
      Serial.printf("Failed (status code = %d)\n", mqttClient.state());
      Serial.printf("Try again in %d miliseconds.\n", retryInterval);
      delay(retryInterval);
    }
  }
}

void connectToWiFi(const char* ssid, const char* password, int retryInterval, bool reconnecting) {
  Serial.printf("%s to WiFi %s", reconnecting ? "Reconnecting" : "Connecting", ssid);

  // Try to connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(WIFI_RETRY_INTERVAL);
    Serial.print(".");
  }
  Serial.print("\nWiFi connected!\n");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void onMotionDetection() {
  motionDetected = millis();
}

void publishWithRetry(char* topic, char* payload, int retryInterval, int maxTries) {
  Serial.printf("Publishing reading to MQTT server (topic = %s, payload = %s) ...\n", topic, payload);

  if (!mqttClient.publish(topic, payload)) {
    Serial.println("Failed!");
  } else {
    Serial.println("Published successfully.");
  }
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);

  // Connect to WiFi
  connectToWiFi(WIFI_SSID, WIFI_PASSWORD, WIFI_RETRY_INTERVAL, false);

  // Configure the MQTT client
  mqttClient.setServer(MQTT_SERVER, MQTT_SERVER_PORT);
  mqttClient.setCallback(receiveCallback);

  // Connect to MQTT server
  connectToMQTT(MQTT_CLIENT_ID, MQTT_MOTION_TOPIC, MQTT_RETRY_INTERVAL, false);
  
  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(MOTION_SENSOR_PIN, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), onMotionDetection, RISING);

  // Warm up motion sensor
  Serial.println("Motion sensor warming up ...");
  delay(MOTION_SENSOR_WARM_UP);
  Serial.println("Motion sensor is ready!");

  Serial.println("=====Setup completed=====");
}

void loop() {
  // If WiFi is disconnected, reconnect
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi(WIFI_SSID, WIFI_PASSWORD, WIFI_RETRY_INTERVAL, true);
  }

  // If MQTT client is disconnected, reconnect
  if (!mqttClient.connected()) {
    connectToMQTT(MQTT_CLIENT_ID, MQTT_MOTION_TOPIC, MQTT_RETRY_INTERVAL, true);
  }

  // Check if motion is detected
  if (motionDetected != NULL) {
    Serial.printf("Motion detected: %d\n", motionDetected);
    
    // Convert reading from ulong to String - allocating a buffer in advance
    char payload[21];
    ulongToChar(motionDetected, payload);

    // Publish reading to MQTT server
    publishWithRetry(MQTT_MOTION_TOPIC, payload, MQTT_RETRY_INTERVAL, MQTT_PUBLISH_MAX_TRIES);

    // Reset reading variable
    motionDetected = NULL;
  }

  // Check if system needs to ping MQTT
  int now = millis();
  if (now - currentTime > MQTT_PING_INTERVAL) {
    currentTime = now;
    publishWithRetry(MQTT_PING_TOPIC, "PING", MQTT_RETRY_INTERVAL, MQTT_PUBLISH_MAX_TRIES);
  }
}