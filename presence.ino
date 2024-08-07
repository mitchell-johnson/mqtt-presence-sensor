//Note this is a cut down version of the config from https://github.com/Wovyn/esp32-ld2410-radar-sensor-mqtt-ota/blob/main/src/esp32-ld2410-radar-sensor-mqtt-ota
//For other devices you might need a different config
#define MONITOR_SERIAL Serial
#define RADAR_SERIAL Serial1
#define RADAR_RX_PIN 32
#define RADAR_TX_PIN 33

#include <ld2410.h>
#include <WiFi.h>
#include <MQTT.h>
#include <ArduinoJson.h>

#define CLIENT_ID "office_presence"
#define DEVICE_NAME "Office Presence"

ld2410 radar;
bool engineeringMode = false;
String command;

uint32_t lastReading = 0;
bool radarConnected = false;

constexpr char ssid[] = "<wifi ssid>";
constexpr char password[] = "<wifi password>";
constexpr char mqtt_server[] = "192.168.1.32";
constexpr char WillTopic[] = "$CONNECTED/" CLIENT_ID;
String stateTopic = "home/office_presence/state";

bool logReads = false;

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
  if(radar.presenceDetected()){
    presence = "ON";
  }

  stateDoc["presence"] = presence;
  // Add other sensor readings as needed

  String payload;
  serializeJson(stateDoc, payload);
  if(logReads){
    Serial.println(F("publishing: "));
    Serial.println(payload.c_str());
  }
  client.publish(stateTopic.c_str(), payload.c_str());
}

void setup(void)
{
  MONITOR_SERIAL.begin(19200); //Feedback over Serial Monitor
  delay(500); //Give a while for Serial Monitor to wake up
  setupWiFi();
  client.begin(mqtt_server, net);
  client.onMessage(mqttCallback);
  //radar.debug(Serial); //Uncomment to show debug information from the library on the Serial Monitor. By default this does not show sensor reads as they are very frequent.

  RADAR_SERIAL.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN); //UART for monitoring the radar

  delay(500);
  MONITOR_SERIAL.print(F("\nConnect LD2410 radar TX to GPIO:"));
  MONITOR_SERIAL.println(RADAR_RX_PIN);
  MONITOR_SERIAL.print(F("Connect LD2410 radar RX to GPIO:"));
  MONITOR_SERIAL.println(RADAR_TX_PIN);
  MONITOR_SERIAL.print(F("LD2410 radar sensor initialising: "));
  if(radar.begin(RADAR_SERIAL))
  {
    MONITOR_SERIAL.println(F("OK"));
    MONITOR_SERIAL.print(F("LD2410 firmware version: "));
    MONITOR_SERIAL.print(radar.firmware_major_version);
    MONITOR_SERIAL.print('.');
    MONITOR_SERIAL.print(radar.firmware_minor_version);
    MONITOR_SERIAL.print('.');
    MONITOR_SERIAL.println(radar.firmware_bugfix_version, HEX);
  }
  else
  {
    MONITOR_SERIAL.println(F("not connected"));
  }
  MONITOR_SERIAL.println(F("Supported commands\nread: read current values from the sensor\nreadconfig: read the configuration from the sensor\nsetmaxvalues <motion gate> <stationary gate> <inactivitytimer>\nsetsensitivity <gate> <motionsensitivity> <stationarysensitivity>\nenableengineeringmode: enable engineering mode\ndisableengineeringmode: disable engineering mode\nrestart: restart the sensor\nreadversion: read firmware version\nfactoryreset: factory reset the sensor\n"));
}

void loop()
{
  radar.read(); //Always read frames from the sensor
  if(MONITOR_SERIAL.available())
  {
    char typedCharacter = MONITOR_SERIAL.read();
    if(typedCharacter == '\r' || typedCharacter == '\n')
    {
      command.trim();
      if(command.equals("logReads"))
      {
        command = "";
        logReads = !logReads;
      } else if(command.equals("read"))
      {
        command = "";
        MONITOR_SERIAL.print(F("Reading from sensor: "));
        if(radar.isConnected())
        {
          MONITOR_SERIAL.println(F("OK"));
          if(radar.presenceDetected())
          {
            if(radar.stationaryTargetDetected())
            {
              MONITOR_SERIAL.print(F("Stationary target: "));
              MONITOR_SERIAL.print(radar.stationaryTargetDistance());
              MONITOR_SERIAL.print(F("cm energy: "));
              MONITOR_SERIAL.println(radar.stationaryTargetEnergy());
            }
            if(radar.movingTargetDetected())
            {
              MONITOR_SERIAL.print(F("Moving target: "));
              MONITOR_SERIAL.print(radar.movingTargetDistance());
              MONITOR_SERIAL.print(F("cm energy: "));
              MONITOR_SERIAL.println(radar.movingTargetEnergy());
            }
          }
          else
          {
            MONITOR_SERIAL.println(F("nothing detected"));
          }
        }
        else
        {
          MONITOR_SERIAL.println(F("failed to read"));
        }
      }
      else if(command.equals("readconfig"))
      {
        command = "";
        MONITOR_SERIAL.print(F("Reading configuration from sensor: "));
        if(radar.requestCurrentConfiguration())
        {
          MONITOR_SERIAL.println(F("OK"));
          MONITOR_SERIAL.print(F("Maximum gate ID: "));
          MONITOR_SERIAL.println(radar.max_gate);
          MONITOR_SERIAL.print(F("Maximum gate for moving targets: "));
          MONITOR_SERIAL.println(radar.max_moving_gate);
          MONITOR_SERIAL.print(F("Maximum gate for stationary targets: "));
          MONITOR_SERIAL.println(radar.max_stationary_gate);
          MONITOR_SERIAL.print(F("Idle time for targets: "));
          MONITOR_SERIAL.println(radar.sensor_idle_time);
          MONITOR_SERIAL.println(F("Gate sensitivity"));
          for(uint8_t gate = 0; gate <= radar.max_gate; gate++)
          {
            MONITOR_SERIAL.print(F("Gate "));
            MONITOR_SERIAL.print(gate);
            MONITOR_SERIAL.print(F(" moving targets: "));
            MONITOR_SERIAL.print(radar.motion_sensitivity[gate]);
            MONITOR_SERIAL.print(F(" stationary targets: "));
            MONITOR_SERIAL.println(radar.stationary_sensitivity[gate]);
          }
        }
        else
        {
          MONITOR_SERIAL.println(F("Failed"));
        }
      }
      else if(command.startsWith("setmaxvalues"))
      {
        uint8_t firstSpace = command.indexOf(' ');
        uint8_t secondSpace = command.indexOf(' ',firstSpace + 1);
        uint8_t thirdSpace = command.indexOf(' ',secondSpace + 1);
        uint8_t newMovingMaxDistance = (command.substring(firstSpace,secondSpace)).toInt();
        uint8_t newStationaryMaxDistance = (command.substring(secondSpace,thirdSpace)).toInt();
        uint16_t inactivityTimer = (command.substring(thirdSpace,command.length())).toInt();
        if(newMovingMaxDistance > 0 && newStationaryMaxDistance > 0 && newMovingMaxDistance <= 8 && newStationaryMaxDistance <= 8)
        {
          MONITOR_SERIAL.print(F("Setting max values to gate "));
          MONITOR_SERIAL.print(newMovingMaxDistance);
          MONITOR_SERIAL.print(F(" moving targets, gate "));
          MONITOR_SERIAL.print(newStationaryMaxDistance);
          MONITOR_SERIAL.print(F(" stationary targets, "));
          MONITOR_SERIAL.print(inactivityTimer);
          MONITOR_SERIAL.print(F("s inactivity timer: "));
          command = "";
          if(radar.setMaxValues(newMovingMaxDistance, newStationaryMaxDistance, inactivityTimer))
          {
            MONITOR_SERIAL.println(F("OK, now restart to apply settings"));
          }
          else
          {
            MONITOR_SERIAL.println(F("failed"));
          }
        }
        else
        {
          MONITOR_SERIAL.print(F("Can't set distances to "));
          MONITOR_SERIAL.print(newMovingMaxDistance);
          MONITOR_SERIAL.print(F(" moving "));
          MONITOR_SERIAL.print(newStationaryMaxDistance);
          MONITOR_SERIAL.println(F(" stationary, try again"));
          command = "";
        }
      }
      else if(command.startsWith("setsensitivity"))
      {
        uint8_t firstSpace = command.indexOf(' ');
        uint8_t secondSpace = command.indexOf(' ',firstSpace + 1);
        uint8_t thirdSpace = command.indexOf(' ',secondSpace + 1);
        uint8_t gate = (command.substring(firstSpace,secondSpace)).toInt();
        uint8_t motionSensitivity = (command.substring(secondSpace,thirdSpace)).toInt();
        uint8_t stationarySensitivity = (command.substring(thirdSpace,command.length())).toInt();
        if(motionSensitivity >= 0 && stationarySensitivity >= 0 && motionSensitivity <= 100 && stationarySensitivity <= 100)
        {
          MONITOR_SERIAL.print(F("Setting gate "));
          MONITOR_SERIAL.print(gate);
          MONITOR_SERIAL.print(F(" motion sensitivity to "));
          MONITOR_SERIAL.print(motionSensitivity);
          MONITOR_SERIAL.print(F(" & stationary sensitivity to "));
          MONITOR_SERIAL.print(stationarySensitivity);
          MONITOR_SERIAL.println(F(": "));
          command = "";
          if(radar.setGateSensitivityThreshold(gate, motionSensitivity, stationarySensitivity))
          {
            MONITOR_SERIAL.println(F("OK, now restart to apply settings"));
          }
          else
          {
            MONITOR_SERIAL.println(F("failed"));
          }
        }
        else
        {
          MONITOR_SERIAL.print(F("Can't set gate "));
          MONITOR_SERIAL.print(gate);
          MONITOR_SERIAL.print(F(" motion sensitivity to "));
          MONITOR_SERIAL.print(motionSensitivity);
          MONITOR_SERIAL.print(F(" & stationary sensitivity to "));
          MONITOR_SERIAL.print(stationarySensitivity);
          MONITOR_SERIAL.println(F(", try again"));
          command = "";
        }
      }
      else if(command.equals("enableengineeringmode"))
      {
        command = "";
        MONITOR_SERIAL.print(F("Enabling engineering mode: "));
        if(radar.requestStartEngineeringMode())
        {
          MONITOR_SERIAL.println(F("OK"));
        }
        else
        {
          MONITOR_SERIAL.println(F("failed"));
        }
      }
      else if(command.equals("disableengineeringmode"))
      {
        command = "";
        MONITOR_SERIAL.print(F("Disabling engineering mode: "));
        if(radar.requestEndEngineeringMode())
        {
          MONITOR_SERIAL.println(F("OK"));
        }
        else
        {
          MONITOR_SERIAL.println(F("failed"));
        }
      }
      else if(command.equals("restart"))
      {
        command = "";
        MONITOR_SERIAL.print(F("Restarting sensor: "));
        if(radar.requestRestart())
        {
          MONITOR_SERIAL.println(F("OK"));
        }
        else
        {
          MONITOR_SERIAL.println(F("failed"));
        }
      }
      else if(command.equals("readversion"))
      {
        command = "";
        MONITOR_SERIAL.print(F("Requesting firmware version: "));
        if(radar.requestFirmwareVersion())
        {
          MONITOR_SERIAL.print('v');
          MONITOR_SERIAL.print(radar.firmware_major_version);
          MONITOR_SERIAL.print('.');
          MONITOR_SERIAL.print(radar.firmware_minor_version);
          MONITOR_SERIAL.print('.');
          MONITOR_SERIAL.println(radar.firmware_bugfix_version,HEX);
        }
        else
        {
          MONITOR_SERIAL.println(F("Failed"));
        }
      }
      else if(command.equals("factoryreset"))
      {
        command = "";
        MONITOR_SERIAL.print(F("Factory resetting sensor: "));
        if(radar.requestFactoryReset())
        {
          MONITOR_SERIAL.println(F("OK, now restart sensor to take effect"));
        }
        else
        {
          MONITOR_SERIAL.println(F("failed"));
        }
      }
      else
      {
        MONITOR_SERIAL.print(F("Unknown command: "));
        MONITOR_SERIAL.println(command);
        command = "";
      }
    }
    else
    {
      command += typedCharacter;
    }
  }

  if(radar.isConnected() && millis() - lastReading > 1000)  //Report every 1000ms
  {
    lastReading = millis();
    publishSensorData();

  if (!client.connected()) {
    connectMqtt();
  }
  
  client.loop();
  
}
