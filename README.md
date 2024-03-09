# RCO2

# Update - Alternative Hardware Support
This branch of the RCO2 project was created to explore supporting alternative hardware configurations, motivated largely by continuing instabilities experienced in using the Adafruit ESP32S3 Reverse TFT
Feather with the Arduino 2.X IDE and associated ESP32 platform support. To maximize compatibility with the original hardware the Adafruit [ESP32 V2 Feather](https://www.adafruit.com/product/5400) and [1.14" 240x135 TFT](https://www.adafruit.com/product/4383) display breakout board served as a good alternative, especially as the ESP32 V2 Feather has always performed perfectly in use with the Arduino 2.X IDE.

This variant of RCO2 started as a barebones buildup from example sketches included with the Arduino libraries for the Sensirion [SCD40 sensor](https://github.com/Sensirion/arduino-i2c-scd4x) and [ST7789 based](https://github.com/adafruit/Adafruit-ST7735-Library) TFT display, combined in an overall structure equivalent to the established RCO2 codebase.  Along the way it allowed exploratory development in several areas, both associated with supporting new (and mulitple simultaneous) hardware plattforms and in optimizing use of the SCD40:
* Use of the SCD40 Low Power Periodic Sampling mode, in which the sensor reports data every thirty seconds rather than the default five seconds.
* Use of the SCD40 IsDataReady flag to only attempt to read and return sensor data when the sensor has stable values to report.
* Make use of the temperature offset capability of the SCD40 to calibrate for the physical circumstances of the deployment environment.  By default the SCD40 presumes it is operating in a setting that is 4C warmer than actual ambient surroundings, which may be accurate in high power periodical sampling mode inside an enclosed case, but is not accurate in low power mode when the sensor is well exposed to its environment.
* Make modifications necessary to fully enable the sensor simulation mode of RCO2, in which no attempt is made to connect to or manage actual sensors and instead reasonable semi-random values are synthesized within the sketch.
* Allow for configurable support of battery monitoring, as different ESP32 Feather devices offer a mixture of on-board monitors ranging from simple analog voltage dividers to sophisticated I2C battery controller ICs.
* Recogize and adapt to target hardware using C++ preprocessor values defined by the board support packages integrated into the Arduino IDE.  This allows the sketch to automatically detect the specific board in use and incorporate code enclosed in suitable C++ #define/#endif blocks. 
* Presume the availabilty of a pushbutton to allow the user to sequence through a collection of supported screens on the TFT display, but also allow configuration of that button to vary depending on the board used.
* Provide some new screens for the user, including one that displays the maximum, minimum and average values observed for CO2, temperature, and humidity.  To simplify managing data gathered from the SCD40, additional code was added to introduce a Measure object that offers easy built-in access to minimum, maximum, and average values gathered over time.  This extended the multi-screen support already incorporated in RCO2.

# Future
Development will continue in this branch to gradually adapat the code so it more fully reflects the nature of RCO2 (e.g., support for DEBUG mode operations).  The goal is to introduce here all the features of the core RCO2 codebase, at which time it would be practical to merge this branch into that main codebase.



# ----- ORIGINAL RCO2 README FOLLOWS BELOW -----

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