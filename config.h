/*
  Project Name:   rco2
  Description:    non-secret configuration data
*/

// Configuration Step 1: Create and/or configure secrets.h. Use secrets_template.h as guide to create secrets.h

// Configuration Step 2: Set debug parameters
// comment out to turn off; 1 = summary, 2 = verbose
#define DEBUG 2

// Configuration Step 3: simulate hardware inputs, returning random but plausible values
// comment out to turn off
// #define HARDWARE_SIMULATE

// Configuration variables that change rarely

// Buttons
const uint8_t buttonD1Pin = 4; // initially LOW
const uint16_t buttonDebounceDelay = 50; // time in milliseconds to debounce button

// Display

// Adafruit Funhouse changes
#define TFT_RST TFT_RESET
#define TFT_BACKLITE TFT_BACKLIGHT
const uint16_t screenWidth = 240;
const uint16_t screenHeight = 240;
const uint8_t screenRotation = 0; // rotation 3 orients 0,0 next to D0 button

// const uint8_t screenRotation = 1; // rotation 3 orients 0,0 next to D0 button
const uint8_t screenCount = 5;

// screen layout assists in pixels
// const uint16_t screenWidth = 135;
// const uint16_t screenHeight = 240;
const uint16_t xMargins = 5;
const uint16_t yMargins = 2;
const uint16_t batteryBarWidth = 28;
const uint16_t batteryBarHeight = 10;

// Battery
const uint16_t batteryVoltageMin = 370; // in V, will be divided by 100.0f to give floats
const uint16_t batteryVoltageMax = 420;
// software battery read (Feather ESP32 V2)
#define BATTERY_VOLTAGE_PIN A13

// Simulation boundary values
#ifdef HARDWARE_SIMULATE
  const uint16_t sensorTempMin =      1500; // in Celcius, divided by 100.0 to give floats
  const uint16_t sensorTempMax =      2500;
  const uint16_t sensorHumidityMin =  500; // in RH%, divided by 100.0 to give floats
  const uint16_t sensorHumidityMax =  9500;
#endif

// CO2 sensor
#ifdef DEBUG
	// time between samples in seconds
  const uint16_t sensorSampleInterval = 30;
#else
  const uint16_t sensorSampleInterval = 60;
#endif

// Define CO2 values that constitute Red (Alarm) & Yellow (Warning) values
// US NIOSH (1987) recommendations:
// 250-350 ppm - normal outdoor ambient concentrations
// 600 ppm - minimal air quality complaints
// 600-1,000 ppm - less clearly interpreted
// 1,000 ppm - indicates inadequate ventilation; complaints such as headaches, fatigue, and eye and throat irritation will be more widespread; 1,000 ppm should be used as an upper limit for indoor levels

const uint16_t co2Warning = 800; // Associated with "OK"
const uint16_t co2Alarm = 1000; // Associated with "Poor"

const String co2Labels[3]={"Good", "So-So", "Poor"};
// Subjective color scheme using 16 bit ('565') RGB colors a la ST77XX display
const uint16_t co2Color[3] = {
    0x07E0,   // GREEN = "Good"
    0xFFE0,   // YELLOW = "So-So"
    0xF800    // RED = "Poor"
  };

const uint16_t sensorCO2Min =      400;   // in ppm
const uint16_t sensorCO2Max =      2000;  // in ppm
const uint16_t sensorTempCOffset = 0;     // in Celcius

// Hardware
// Sleep time in seconds if hardware error occurs
const uint8_t hardwareRebootInterval = 10;