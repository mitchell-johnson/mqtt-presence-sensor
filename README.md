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
   - Connect the RX pin of the radar sensor to GPIO 33 of the ESP32.
   - Connect the TX pin of the radar sensor to GPIO 32 of the ESP32.
   - Connect the power and ground pins of the radar sensor to the 3.3V and GND pins of the ESP32 respectively.
  
Note that you will likely need to change the sensitivity threasholds in the radar to get it to function well in your envronment. 
Future plans are to have that configurable via MQTT

For my setup in my office I have the settings (readconfig command)

Gate sensitivity
Gate 0 moving targets: 50 stationary targets: 50
Gate 1 moving targets: 40 stationary targets: 40
Gate 2 moving targets: 40 stationary targets: 40
Gate 3 moving targets: 40 stationary targets: 40
Gate 4 moving targets: 40 stationary targets: 40
Gate 5 moving targets: 90 stationary targets: 90
Gate 6 moving targets: 100 stationary targets: 100
Gate 7 moving targets: 100 stationary targets: 100
Gate 8 moving targets: 100 stationary targets: 100

The Gates 0-8 are 75cm thresholds starting immediately in front of the device. 100 == No sensing.



![image](https://github.com/user-attachments/assets/3f827ae9-2509-46b2-b17b-07bac7e267ce)

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

Finished Product 
![img_2796 Large](https://github.com/user-attachments/assets/5c997e6c-e718-4904-b837-b9ebf7bfd01d)
![img_2795 Large](https://github.com/user-attachments/assets/e85794e9-687a-4421-8774-d2fc54b77c84)
![img_2822 Large](https://github.com/user-attachments/assets/83db2fd9-5380-4779-a72e-d675307ac8f2)


