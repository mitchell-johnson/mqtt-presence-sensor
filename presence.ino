// Define implementation-specific variables
#if defined(ESP32)
  #ifdef ESP_IDF_VERSION_MAJOR // IDF 4+
    #if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
      constexpr auto MONITOR_SERIAL = Serial;
      constexpr auto RADAR_SERIAL = Serial1;
      constexpr int RADAR_RX_PIN = 32;
      constexpr int RADAR_TX_PIN = 33;
    #elif CONFIG_IDF_TARGET_ESP32S2
      constexpr auto MONITOR_SERIAL = Serial;
      constexpr auto RADAR_SERIAL = Serial1;
      constexpr int RADAR_RX_PIN = 9;
      constexpr int RADAR_TX_PIN = 8;
    #elif CONFIG_IDF_TARGET_ESP32C3
      constexpr auto MONITOR_SERIAL = Serial;
      constexpr auto RADAR_SERIAL = Serial1;
      constexpr int RADAR_RX_PIN = 4;
      constexpr int RADAR_TX_PIN = 5;
    #else 
      #error Target CONFIG_IDF_TARGET is not supported
    #endif
  #else // ESP32 Before IDF 4.0
    constexpr auto MONITOR_SERIAL = Serial;
    constexpr auto RADAR_SERIAL = Serial1;
    constexpr int RADAR_RX_PIN = 32;
    constexpr int RADAR_TX_PIN = 33;
  #endif
#elif defined(__AVR_ATmega32U4__)
  constexpr auto MONITOR_SERIAL = Serial;
  constexpr auto RADAR_SERIAL = Serial1;
  constexpr int RADAR_RX_PIN = 0;
  constexpr int RADAR_TX_PIN = 1;
#endif

// WiFi and MQTT configuration
constexpr char ssid[] = "<WIFI SSID>";
constexpr char password[] = "<WIFI Password>";
constexpr char mqtt_server[] = "192.168.1.32";
constexpr char CLIENT_ID[] = "office_presence";
constexpr char DEVICE_NAME[] = "Office Presence";
constexpr char WillTopic[] = "$CONNECTED/" CLIENT_ID;
String stateTopic = "home/office_presence/state";

int stillThreshold = 30;
int movingThreshold = 30;

// Include necessary libraries
#include <ld2410.h>
#include <WiFi.h>
#include <MQTT.h>
#include <ArduinoJson.h>

// Initialize radar and flags
ld2410 radar;
bool engineeringMode = false;
String command;

uint32_t lastReading = 0;
bool radarConnected = false;

// Initializing WiFi and MQTT client
WiFiClient net;
MQTTClient client;

// Function to set up WiFi connection
void setupWiFi() {
  Serial.println("Connecting to " + String(ssid));
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());
}

void connectMqtt (){
  Serial.print("\nconnecting mqtt...");
  while (!client.connected()) {
    client.setWill(WillTopic);
    digitalWrite(LED_BUILTIN, LOW);
    if (client.connect(CLIENT_ID)) {
      Serial.println("connected");
      digitalWrite(LED_BUILTIN, HIGH);
      publishAutoDiscoveryConfig();
    } else {
      Serial.print(".");
      delay(1000);
    }
  }
  Serial.println("\n mqtt connected!");
  client.subscribe("/hello");
}

void publishAutoDiscoveryConfig() {
  const char* baseTopic = "homeassistant/sensor/";
  String clientId = CLIENT_ID; // Use your actual client ID

  // Moving Distance
  {
    String configTopic = String(baseTopic) + clientId + "/moving_distance/config";
    DynamicJsonDocument configDoc(1024);
    configDoc["~"] = "home/" + clientId + "/moving_distance";
    configDoc["name"] = clientId + " Moving Distance";
    configDoc["uniq_id"] = clientId + "_moving_distance";
    configDoc["stat_t"] = stateTopic;
    configDoc["unit_of_meas"] = "cm";
    configDoc["device_class"] = "distance";
    configDoc["value_template"] = "{{ value_json.moving_distance }}";
    JsonObject device = configDoc.createNestedObject("device");
    JsonArray device_identifiers = device.createNestedArray("identifiers");
    device_identifiers.add(CLIENT_ID);
    device["name"] = DEVICE_NAME;
    String payload;
    serializeJson(configDoc, payload);
    client.publish(configTopic.c_str(), payload.c_str());
  }

  // Still Distance
  {
    String configTopic = String(baseTopic) + clientId + "/still_distance/config";
    DynamicJsonDocument configDoc(1024);
    configDoc["~"] = "home/" + clientId + "/still_distance";
    configDoc["name"] = clientId + " Still Distance";
    configDoc["uniq_id"] = clientId + "_still_distance";
    configDoc["stat_t"] = stateTopic;
    configDoc["unit_of_meas"] = "cm";
    configDoc["device_class"] = "distance";
    configDoc["value_template"] = "{{ value_json.still_distance }}";
    JsonObject device = configDoc.createNestedObject("device");
    JsonArray device_identifiers = device.createNestedArray("identifiers");
    device_identifiers.add(CLIENT_ID);
    device["name"] = DEVICE_NAME;
    String payload;
    serializeJson(configDoc, payload);
    client.publish(configTopic.c_str(), payload.c_str());
  }

  // Moving Energy
  {
    String configTopic = String(baseTopic) + clientId + "/moving_energy/config";
    DynamicJsonDocument configDoc(1024);
    configDoc["~"] = "home/" + clientId + "/moving_energy";
    configDoc["name"] = clientId + " Moving Engergy";
    configDoc["uniq_id"] = clientId + "_moving_energy";
    configDoc["stat_t"] = stateTopic;
    configDoc["unit_of_meas"] = "%";
    configDoc["device_class"] = "energy";
    configDoc["value_template"] = "{{ value_json.moving_energy }}";
    JsonObject device = configDoc.createNestedObject("device");
    JsonArray device_identifiers = device.createNestedArray("identifiers");
    device_identifiers.add(CLIENT_ID);
    device["name"] = DEVICE_NAME;
    String payload;
    serializeJson(configDoc, payload);
    client.publish(configTopic.c_str(), payload.c_str());
  }

  // Still Energy
  {
    String configTopic = String(baseTopic) + clientId + "/still_energy/config";
    DynamicJsonDocument configDoc(1024);
    configDoc["~"] = "home/" + clientId;
    configDoc["name"] = clientId + " Still Energy";
    configDoc["uniq_id"] = clientId + "_still_energy";
    configDoc["stat_t"] = stateTopic;
    configDoc["unit_of_meas"] = "%";
    configDoc["device_class"] = "energy";
    configDoc["value_template"] = "{{ value_json.still_energy }}";
    JsonObject device = configDoc.createNestedObject("device");
    JsonArray device_identifiers = device.createNestedArray("identifiers");
    device_identifiers.add(CLIENT_ID);
    device["name"] = DEVICE_NAME;
    String payload;
    serializeJson(configDoc, payload);
    client.publish(configTopic.c_str(), payload.c_str());
  }

  // Person is there
  {
    String configTopic = String("homeassistant/binary_sensor/") + clientId + "/presence/config";
    DynamicJsonDocument configDoc(1024);
    configDoc["~"] = "home/" + clientId;
    configDoc["name"] = clientId + " Presence";
    configDoc["uniq_id"] = clientId + "_presence";
    configDoc["stat_t"] = stateTopic;
    configDoc["device_class"] = "occupancy";
    configDoc["value_template"] = "{{ value_json.presence }}";
    JsonObject device = configDoc.createNestedObject("device");
    JsonArray device_identifiers = device.createNestedArray("identifiers");
    device_identifiers.add(CLIENT_ID);
    device["name"] = DEVICE_NAME;
    device["manufacturer"] = "Mitchell";
    String payload;
    serializeJson(configDoc, payload);
    client.publish(configTopic.c_str(), payload.c_str());
  }
}

// MQTT callback function to handle incoming messages
void mqttCallback(String &topic, String &payload) {
  Serial.print("Got MQTT Message...");
  
}

void publishSensorData() {
  String clientId = CLIENT_ID; // Use your actual client ID
  String stateTopic = "home/" + clientId + "/state";
  radar.read();

  DynamicJsonDocument stateDoc(1024);
  // Populate stateDoc with actual sensor readings
  stateDoc["moving_distance"] = radar.movingTargetDistance();
  stateDoc["still_distance"] = radar.stationaryTargetDistance();
  stateDoc["moving_energy"] = radar.movingTargetEnergy();
  stateDoc["still_energy"] = radar.stationaryTargetEnergy();

  //binary sensor to indicate if someone is present or not
  String presence = "OFF";
  if(radar.movingTargetEnergy() > movingThreshold || radar.stationaryTargetEnergy() > movingThreshold){
    presence = "ON";
  }

  stateDoc["presence"] = presence;
  // Add other sensor readings as needed

  String payload;
  serializeJson(stateDoc, payload);
  Serial.println(F("publishing: "));
  Serial.print(payload.c_str());
  client.publish(stateTopic.c_str(), payload.c_str());
}

void setup(void)
{
  MONITOR_SERIAL.begin(19200); //Feedback over Serial Monitor
  delay(5000); //5 second delay before startup
  pinMode(LED_BUILTIN, OUTPUT);
  WiFi.begin(ssid, password);
  client.begin(mqtt_server, net);
  client.onMessage(mqttCallback);
  connectMqtt();
  setupWiFi();
  radar.begin(RADAR_SERIAL, RADAR_RX_PIN, RADAR_TX_PIN);
  MONITOR_SERIAL.println("setting up device");

  client.subscribe("/hello");
}

void loop(void)
{
  // Maintain WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    setupWiFi();
  }

  // Maintain MQTT connection
  if (!client.connected()) {
    connectMqtt();
  }
  client.loop();

  // Publish sensor data at regular intervals
  if (millis() - lastReading >= 5000) { // Change the interval as needed
    publishSensorData();
    lastReading = millis();
  }
}
