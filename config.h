/*
  Project Name:   realtime_co2
  Description:    non-secret configuration data
*/

// Configuration Step 1: Create and/or configure secrets.h. Use secrets_template.h as guide to create secrets.h

// Configuration Step 2: Set debug parameters
// comment out to turn off; 1 = summary, 2 = verbose
// #define DEBUG 2

// Configuration Step 3: simulate hardware inputs, returning random but plausible values
// comment out to turn off
// #define SENSOR_SIMULATE

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

// Configuration variables that change rarely

// Display
const uint8_t displayRotation = 3; // rotation 3 orients 0,0 next to D0 button

// Battery
const float batteryVoltageMinAlert = 3.7;
const float batteryVoltageMaxAlert = 4.2;

// CO2 
//sample timing
#ifdef DEBUG
	// number of times SCD40 is read, last read is the sample value
	const uint8_t sensorReadsPerSample =	1;
	// time between samples in seconds. Must be >=180 to protect 3 color EPD
  const uint16_t sensorSampleInterval = 60;
#else
  const uint8_t sensorReadsPerSample =  3;
  const uint16_t sensorSampleInterval = 180;
#endif
const String co2Labels[5]={"Good", "OK", "So-So", "Poor", "Bad"};

// Hardware
// Sleep time in seconds if hardware error occurs
const uint8_t hardwareRebootInterval = 10;