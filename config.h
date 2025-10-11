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
#define HARDWARE_SIMULATE

// Configuration variables that change rarely

// Buttons
const uint8_t buttonD1Pin = 1; // initially LOW
const uint16_t buttonDebounceDelay = 50; // time in milliseconds to debounce button

// Display
const uint8_t screenRotation = 1; // rotation 3 orients 0,0 next to D0 button
const uint8_t screenCount = 5;

// screen layout assists in pixels
const uint16_t xMargins = 5;
const uint16_t yMargins = 2;
const uint16_t batteryBarWidth = 28;
const uint16_t batteryBarHeight = 10;

// Battery
const uint16_t batteryVoltageMin = 370; // in V, will be divided by 100.0f to give floats
const uint16_t batteryVoltageMax = 420;

// Simulation boundary values
#ifdef HARDWARE_SIMULATE
  const uint16_t  sensorTempMinF =       14; // -10C per datasheet
  const uint16_t  sensorTempMaxF =       140; // 60C per datasheet
  const uint16_t  sensorHumidityMin =    0; // RH% per datasheet
  const uint16_t  sensorHumidityMax =    100;
  const uint8_t   sensorCO2VariabilityRange = 30;
#endif

// CO2 sensor
#ifdef DEBUG
	// time between samples in seconds
  const uint16_t sensorSampleInterval = 30;
#else
  const uint16_t sensorSampleInterval = 60;
#endif

// CO2 value thresholds for labeling
const uint16_t co2Fair =  800;
const uint16_t co2Poor =  1200;
const uint16_t co2Bad =   1600;

// warnings
const String warningLabels[4]={"Good", "Fair", "Poor", "Bad"};
// Subjective color scheme using 16 bit ('565') RGB colors
const uint16_t warningColor[4] = {
  0x07E0, // Green = "Good"
  0xFFE0, // Yellow = "Fair"
  0xFD20, // Orange = "Poor"
  0xF800  // Red = "Bad"
};

const uint16_t sensorCO2Min =      400;   // in ppm
const uint16_t sensorCO2Max =      2000;  // in ppm
const uint8_t co2SensorReadFailureLimit = 20;

// Hardware
// Sleep time in seconds if hardware error occurs
const uint8_t hardwareRebootInterval = 10;