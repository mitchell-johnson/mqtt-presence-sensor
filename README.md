# Office Presence Detector with ESP32 and LD2410 Radar

## Project Overview

The Office Presence Detector is a smart device designed to monitor occupancy in an office space using the LD2410 radar sensor and an ESP32 microcontroller. This device connects to a Wi-Fi network, publishes sensor data to an MQTT server, and integrates with Home Assistant for real-time occupancy monitoring. The system can detect presence based on moving and stationary energy thresholds, providing detailed distance and energy readings for both moving and stationary targets.

## Features

- **Wi-Fi Connectivity:** Connects to a Wi-Fi network to communicate with an MQTT server.
- **MQTT Integration:** Publishes sensor data to an MQTT server for integration with smart home systems like Home Assistant.
- **LD2410 Radar Sensor:** Utilizes the LD2410 radar sensor to detect presence and measure distance and energy of moving and stationary targets.
- **Auto Discovery for Home Assistant:** Automatically publishes configuration data for Home Assistant, allowing seamless integration and easy setup.
- **LED Indicator:** Built-in LED for visual status indication.
- **Customizable Thresholds:** Allows customization of thresholds for still and moving energy to determine presence.

## Hardware Requirements

- ESP32 Development Board
- LD2410 Radar Sensor
- LED (built-in on most ESP32 boards)
- Breadboard and Jumper Wires (for prototyping)
- Wi-Fi Network with internet access
- MQTT Server (e.g., Mosquitto)

## Software Requirements

- Arduino IDE
- ArduinoJson library
- MQTT library
- ld2410 library (specific to the radar sensor)

## Pin Configuration

| ESP32 Pin | Function         |
|-----------|------------------|
| 32        | RADAR_RX_PIN     |
| 33        | RADAR_TX_PIN     |
| 2         | LED_BUILTIN      |

## Getting Started

### Step 1: Hardware Setup

1. Connect the LD2410 radar sensor to the ESP32:
   - Connect the RX pin of the radar sensor to GPIO 32 of the ESP32.
   - Connect the TX pin of the radar sensor to GPIO 33 of the ESP32.
   - Connect the power and ground pins of the radar sensor to the 3.3V and GND pins of the ESP32 respectively.

### Step 2: Software Setup

1. Install the Arduino IDE from [arduino.cc](https://www.arduino.cc/en/software).
2. Open the Arduino IDE and install the necessary libraries:
   - Go to `Sketch` > `Include Library` > `Manage Libraries...`.
   - Search for and install the following libraries:
     - **ArduinoJson**
     - **MQTT**
     - **ld2410** (or the library specific to your radar sensor)
3. Open a new sketch in the Arduino IDE and copy the provided code into the sketch.

### Step 3: Configure the Code

1. Update the Wi-Fi and MQTT configuration variables in the code with your network credentials and MQTT server details:

    ```cpp
    constexpr char ssid[] = "Your_SSID";
    constexpr char password[] = "Your_PASSWORD";
    constexpr char mqtt_server[] = "Your_MQTT_Server_IP";
    constexpr char CLIENT_ID[] = "office_presence";
    constexpr char DEVICE_NAME[] = "Office Presence";
    ```

2. Customize the thresholds for still and moving energy if needed:

    ```cpp
    int stillThreshold = 30;
    int movingThreshold = 30;
    ```

### Step 4: Upload the Code

1. Connect your ESP32 to your computer using a USB cable.
2. Select the correct board and port in the Arduino IDE:
   - Go to `Tools` > `Board` and select `ESP32 Dev Module`.
   - Go to `Tools` > `Port` and select the appropriate port.
3. Click the `Upload` button to upload the code to your ESP32.

### Step 5: Monitor Serial Output

1. Open the Serial Monitor in the Arduino IDE (`Tools` > `Serial Monitor`).
2. Set the baud rate to `19200`.
3. Observe the output to ensure the device is connecting to Wi-Fi and the MQTT server correctly.

### Step 6: Integrate with Home Assistant

1. Ensure your MQTT server is running and accessible.
2. Autodiscovery should work and it will show up as a device/entity in HA.
3. For debugging I reccomend MQTT Explorer app.

## Detailed Code Explanation

### Variable Definitions

The code begins by defining implementation-specific variables for different microcontroller targets (ESP32 and ATmega32U4):

```cpp
#if defined(ESP32)
  #ifdef ESP_IDF_VERSION_MAJOR // IDF 4+
    ...
#elif defined(__AVR_ATmega32U4__)
  ...
#endif
```

### Wi-Fi and MQTT Configuration

Wi-Fi and MQTT server details are defined:

```cpp
constexpr char ssid[] = "Your_SSID";
constexpr char password[] = "Your_PASSWORD";
constexpr char mqtt_server[] = "Your_MQTT_Server_IP";
```

### Setup Function

The `setup()` function initializes the serial communication, connects to Wi-Fi, sets up MQTT, and initializes the radar sensor:

```cpp
void setup(void) {
  MONITOR_SERIAL.begin(19200); //Feedback over Serial Monitor
  ...
  radar.begin(RADAR_SERIAL, RADAR_RX_PIN, RADAR_TX_PIN);
  ...
}
```

### Loop Function

The `loop()` function maintains Wi-Fi and MQTT connections and publishes sensor data at regular intervals:

```cpp
void loop(void) {
  ...
  // Publish sensor data at regular intervals
  if (millis() - lastReading >= 5000) {
    publishSensorData();
    lastReading = millis();
  }
}
```

### Function Implementations

#### Wi-Fi Setup

The `setupWiFi()` function connects the ESP32 to the Wi-Fi network:

```cpp
void setupWiFi() {
  Serial.println("Connecting to " + String(ssid));
  ...
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  ...
}
```

#### MQTT Setup

The `connectMqtt()` function connects to the MQTT server and subscribes to relevant topics:

```cpp
void connectMqtt () {
  Serial.print("\nconnecting mqtt...");
  ...
  while (!client.connected()) {
    ...
    if (client.connect(CLIENT_ID)) {
      Serial.println("connected");
      ...
      publishAutoDiscoveryConfig();
    } else {
      Serial.print(".");
      delay(1000);
    }
  }
  ...
}
```

#### Publish Sensor Data

The `publishSensorData()` function reads data from the radar sensor and publishes it to the MQTT server:

```cpp
void publishSensorData() {
  String clientId = CLIENT_ID;
  String stateTopic = "home/" + clientId + "/state";
  radar.read();
  ...
  String presence = "OFF";
  if(radar.movingTargetEnergy() > movingThreshold || radar.stationaryTargetEnergy() > movingThreshold){
    presence = "ON";
  }
  ...
  String payload;
  serializeJson(stateDoc, payload);
  Serial.println(F("publishing: "));
  Serial.print(payload.c_str());
  client.publish(stateTopic.c_str(), payload.c_str());
}
```

### MQTT Callback

The `mqttCallback()` function handles incoming MQTT messages:

```cpp
void mqttCallback(String &topic, String &payload) {
  Serial.print("Got MQTT Message...");
}
```

### Auto Discovery Config for Home Assistant

The `publishAutoDiscoveryConfig()` function publishes the auto-discovery configuration for Home Assistant:

```cpp
void publishAutoDiscoveryConfig() {
  const char* baseTopic = "homeassistant/sensor/";
  String clientId = CLIENT_ID;
  ...
  // Moving Distance
  {
    ...
    String payload;
    serializeJson(configDoc, payload);
    client.publish(configTopic.c_str(), payload.c_str());
  }
  ...
}
```

## Conclusion

This project provides a comprehensive solution for monitoring office presence using an ESP32 and LD2410 radar sensor. The detailed integration with MQTT and Home Assistant allows for seamless real-time monitoring and smart home automation. Customize the code and thresholds as needed to fit your specific requirements and environment. Enjoy building and enhancing your smart office presence detector!
