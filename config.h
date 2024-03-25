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
// #define SENSOR_SIMULATE

// Configuration variables that change rarely

// Buttons
const uint8_t buttonD1Pin = 1; // initially LOW

const int buttonDebounceDelay = 50; // time in milliseconds to debounce button

// Display
const uint8_t screenRotation = 3; // rotation 3 orients 0,0 next to D0 button
const uint8_t screenCount = 5;

// screen layout assists
const uint16_t xMargins = 5;
const uint16_t yMargins = 2;
const uint16_t batteryBarWidth = 28;
const uint16_t batteryBarHeight = 10;

// Battery
const float batteryVoltageMinAlert = 3.7;
const float batteryVoltageMaxAlert = 4.2;

// Simulation values
#ifdef SENSOR_SIMULATE
  const uint16_t sensorTempMin =      1500; // will be divided by 100.0 to give floats
  const uint16_t sensorTempMax =      2500;
  const uint16_t sensorHumidityMin =  500; // will be divided by 100.0 to give floats
  const uint16_t sensorHumidityMax =  9500;
  const uint16_t sensorCO2Min =       400;
  const uint16_t sensorCO2Max =       3000;

  const uint16_t batterySimVoltageMin = 370; // will be divided by 100.0 to give floats
  const uint16_t batterySimVoltageMax = 420;
#endif

// CO2 sensor
//sample timing
#ifdef DEBUG
	// number of times SCD40 is read, last read is the sample value
	const uint8_t  sensorReadsPerSample =	1;
	// time between samples in seconds
  const uint16_t sensorSampleInterval = 60;
#else
  const uint8_t   sensorReadsPerSample =  3;
  const uint16_t  sensorSampleInterval = 120;
#endif
const String co2Labels[5]={"Good", "OK", "So-So", "Poor", "Bad"};
// Subjective color scheme using 16 bit ('565') RGB colors a la ST77XX display
const uint16_t co2Highlight[5] = {
    0x07E0,   // GREEN = "Good"
    0x07E0,   // GREEN = "OK"
    0xFFE0,   // YELLOW = "So-So"
    0xFC00,   // ORANGE = "Poor"
    0xF800    // RED = "Bad"
  };

// Hardware
// Sleep time in seconds if hardware error occurs
const uint8_t hardwareRebootInterval = 10;