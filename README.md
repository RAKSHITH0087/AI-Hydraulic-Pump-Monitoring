# AI-Based Hydraulic Pump Fault Classification

## Project Overview

This project explores hydraulic pump condition monitoring using vibration and temperature sensor data. It aims to support early fault identification and predictive maintenance.

## Hardware Components

* ESP32 microcontroller
* MPU6050 accelerometer and gyroscope
* DS18B20 temperature sensor
* SH1106 OLED display
* Hydraulic pump prototype

## Software and Technologies

* Arduino IDE
* Embedded C/C++ for ESP32
* Sensor data acquisition
* Digital vibration-signal filtering
* Wi-Fi and HTTP communication with a backend API
* Machine learning for fault classification, currently under development

## How It Works

1. The MPU6050 collects acceleration and gyroscope measurements.
2. The DS18B20 measures temperature.
3. The ESP32 processes vibration readings using a digital band-pass filter.
4. Sensor readings are displayed on the OLED screen and serial monitor.
5. When configured and connected, the ESP32 sends sensor data to a backend API over Wi-Fi.

flowchart TD
    A["Hydraulic Pump"] --> B["Vibration Sensor"]
    A --> C["Temperature Sensor"]
    B --> D["ESP32"]
    C --> D
    D --> E["Sensor Data Collection"]
    E --> F["Data Analysis and Feature Extraction"]
    F --> G["ML Model - In Development"]
    G --> H["Normal / Abnormal / Critical"]

## Current Scope

The hydraulic pump monitoring prototype is currently under development. The ESP32-based system collects vibration and temperature data using the MPU6050 accelerometer and DS18B20 temperature sensor.

We are currently working on developing and training the AI/ML model for hydraulic pump fault classification.

Our ongoing work focuses on analysing sensor data, improving the classification approach, and exploring the use of combined vibration and temperature measurements to identify pump conditions as Normal, Abnormal, and Critical.

Further development and testing are planned to improve the system's reliability and its potential application in industrial predictive maintenance.


## Setup

1. Install Arduino IDE and the ESP32 board support package.
2. Install the OneWire, DallasTemperature, and U8g2 libraries.
3. Configure Wi-Fi credentials and backend API settings privately.
4. Connect the sensors according to the firmware pin configuration.
5. Select the appropriate ESP32 board and upload the firmware.

## Intended Application

The project explores a low-cost approach to hydraulic pump condition monitoring and early fault detection to support industrial predictive maintenance.

What Makes This Project Different?

Comparative multisensor analysis: The project aims to compare vibration-only data with combined vibration and temperature data to investigate their usefulness for hydraulic pump fault classification.

Embedded monitoring: An ESP32-based prototype collects sensor measurements, providing a foundation for a compact monitoring system.

Predictive maintenance potential: The proposed three-level classification approach aims to support early investigation of abnormal pump behaviour.

