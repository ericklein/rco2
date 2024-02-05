# RCO2

### Purpose
Realtime CO2, aka rco2, samples and displays temperature, humidity, and CO2 (carbon dioxide) levels using different information screens.

### Features

### Target configuration
- WiFi SSID and password are contained in a `secrets.h` file that is not included in this repo.  Instead you'll find the file `secrets_template.h`, which should be copied to `secrets.h` and then edited to supply the right access credentials and configuration values to match your deployment environment.
- See config.h for parameter configuration

### Bill of Materials (BOM)
- MCU
    - ESP32
- WiFi
    - Supported hardware
        - ESP32 based boards
- environment sensor
    - [SCD40 True CO2, Temperature and Humidity Sensor](https://www.adafruit.com/product/5187)
- Battery
    -
- Screen
    -
### Pinouts
- SPDT switch (on/off)
    - MCU EN to SPD rightmost pin
    - MCU GND to SPD
- SCD40
    - Stemma QT cable between MCU board and SCD40 board

### Issues and Feature Requests
- See GitHub Issues for project

### .plan (big ticket items)
- WiFI Manager support
- OTA Firmware update