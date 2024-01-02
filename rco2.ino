/*
  Project:      rco2
  Description:  Regularly sample temperature, humidity, and co2 levels

  See README.md for target information
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT
#include "secrets.h"

// screen support
#include <Adafruit_SH110X.h>
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

#include "Fonts/meteocons16pt7b.h"
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>

// Special glyphs for the UI
#include "glyphs.h"

// initialize scd40 CO2 sensor
#ifndef SENSOR_SIMULATE
  #include <SensirionI2CScd4x.h>
  SensirionI2CScd4x envSensor;
#endif

// Battery voltage sensor
#ifndef SENSOR_SIMULATE
  #include <Adafruit_LC709203F.h>
  Adafruit_LC709203F lc;
#endif

// screen layout assists
const uint16_t xMargins = 10;
const uint16_t yMargins = 2;
const uint16_t yCO2 = 50;
// const uint16_t ySparkline = 95;
const uint16_t yTemperature = 170;
// const uint16_t sparklineHeight = 40;
const uint16_t batteryBarWidth = 28;
const uint16_t batteryBarHeight = 10;

// environment sensor data
typedef struct {
  float     ambientTemperatureF;
  float     ambientHumidity;     // RH [%]  
  uint16_t  ambientCO2;
} envData;
envData sensorData;

// hardware status data
typedef struct {
  float   batteryPercent;
  float   batteryVoltage;
  uint8_t rssi;
} hdweData;
hdweData hardwareData;

unsigned long prevSampleMs  = 0;  // Timestamp for measuring elapsed sample time

void setup()
{
  // handle Serial first so debugMessage() works
  #ifdef DEBUG
    Serial.begin(115200);
    // wait for serial port connection
    while (!Serial);
  #endif

  debugMessage(String("RCO2 start with ") + SAMPLE_INTERVAL + " second sample interval",1);

  // initiate first to display hardware error messages
  if(!display.begin(0x3C, true)) {
    debugMessage("Display initialization failed",1);
    while(1); // Don't proceed, loop forever
  }
  display.setRotation(displayRotation);

  // Initialize environmental sensor
  if (!sensorCO2Init()) {
    debugMessage("Environment sensor failed to initialize",1);
    screenAlert(5, ((display.height()/2)+6), "No SCD40");
    // This error often occurs right after a firmware flash and reset.
    // Hardware deep sleep typically resolves it, so quickly cycle the hardware
    //powerDisable(HARDWARE_ERROR_INTERVAL);
  }
}

void loop()
{
  // read sensor if time
  // update the screen

  // update current timer value
  unsigned long currentMillis = millis();

  // is it time to read the sensor?
  if((currentMillis - prevSampleMs) >= (SAMPLE_INTERVAL * 1000)) // converting SAMPLE_INTERVAL into milliseconds
  {
    // Check for a battery, and if so, is it supplying enough voltage to drive the SCD40
    batteryRead(batteryReadsPerSample);
    if ((hardwareData.batteryVoltage < batteryVoltageTable[4]) && (hardwareData.batteryVoltage != 0.0))
    {
      debugMessage("Battery below required threshold, rebooting",1);
      screenAlert(20, ((display.height()/2)+6), "Low battery");
      // this is a recursive boot sequence
      // powerDisable(HARDWARE_ERROR_INTERVAL);
  }
  if (!sensorCO2Read())
  {
    debugMessage("SCD40 returned no/bad data",1);
    screenAlert(5, ((display.height()/2)+6),"SCD40 read issue");
    // powerDisable(HARDWARE_ERROR_INTERVAL);
  }
  screenInfo("");
  prevSampleMs = currentMillis;
  }
}


void debugMessage(String messageText, int messageLevel)
// wraps Serial.println as #define conditional
{
  #ifdef DEBUG
    if (messageLevel <= DEBUG)
    {
      Serial.println(messageText);
      Serial.flush();  // Make sure the message gets output (before any sleeping...)
    }
  #endif
}

void screenAlert(int initialX, int initialY, String messageText)
// Display critical error message on screen
{
  debugMessage("screenAlert refresh started",1);
  // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.setTextColor(SH110X_WHITE);
  display.setFont(&FreeSans12pt7b);
  display.setCursor(initialX, initialY);
  display.print(messageText);
  display.display();
  debugMessage("screenAlert refresh complete",1);
}

void screenInfo(String messageText)
// Display environmental information
{
  debugMessage("screenInfo refresh started",1);
  
  // Clear the buffer.
  display.clearDisplay();

  // screen helper routines
  // display battery level in the lower right corner, -3 in first parameter accounts for battery nub
  screenHelperBatteryStatus((display.width()-xMargins-batteryBarWidth-3),(display.height()-yMargins-batteryBarHeight), batteryBarWidth, batteryBarHeight);

  display.setTextColor(SH110X_WHITE);
  display.setFont();
  display.setCursor(0,0);
  display.println(String("Temp is ")+sensorData.ambientTemperatureF+"F");
  display.println(String("Humidity is ")+sensorData.ambientHumidity+"%");
  display.println(String("CO2 is ")+sensorData.ambientCO2+"ppm");
  display.display();

  // // display sparkline
  // screenHelperSparkLine(xMargins,ySparkline,(display.width() - (2* xMargins)),sparklineHeight);

  // // Indoor CO2 level
  // // calculate CO2 value range in 400ppm bands
  // uint8_t co2range = ((sensorData.ambientCO2 - 400) / 400);
  // co2range = constrain(co2range,0,4); // filter CO2 levels above 2400

  // display.setFont(&FreeSans18pt7b);
  // display.setCursor(xMargins, yCO2);
  // display.print("CO");
  // display.setCursor(xMargins+65,yCO2);
  // display.print(": " + String(co2Labels[co2range]));
  // display.setFont(&FreeSans12pt7b);
  // display.setCursor(xMargins+50,yCO2+10);
  // display.print("2");
  // display.setCursor((xMargins+90),yCO2+25);
  // display.print("(" + String(sensorData.ambientCO2) + ")");

  // // Indoor temp
  // int temperatureF = sensorData.ambientTemperatureF + 0.5;
  // display.setFont(&FreeSans18pt7b);
  // if(temperatureF < 100)
  // {
  //   display.setCursor(xMargins,yTemperature);
  //   display.print(String(temperatureF));
  //   display.drawBitmap(xMargins+42,yTemperature-21,epd_bitmap_temperatureF_icon_sm,20,28,SSD1306_WHITE);
  // }
  // else 
  // {
  //   display.setCursor(xMargins,yTemperature);
  //   display.print(String(temperatureF));
  //   display.setFont(&FreeSans12pt7b);
  //   display.setCursor(xMargins+65,yTemperature);
  //   display.print("F"); 
  // }

  // // Indoor humidity
  // display.setFont(&FreeSans18pt7b);
  // display.setCursor(display.width()/2, yTemperature);
  // display.print(String((int)(sensorData.ambientHumidity + 0.5)));
  // display.drawBitmap(display.width()/2+42,yTemperature-21,epd_bitmap_humidity_icon_sm4,20,28,SSD1306_WHITE);

  debugMessage("screenInfo refresh complete",1);
}

void batteryRead(uint8_t reads)
// sets global battery values from i2c battery monitor or analog pin value on supported boards
{
  #ifdef SENSOR_SIMULATE
    batterySimulate();
    debugMessage(String("SIMULATED Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%",1);
  #else
    hardwareData.batteryVoltage = 0.0;  // 0.0 means no battery attached, now try to sample battery voltage
    hardwareData.batteryPercent = 0.0;
    // Sample i2c battery monitor if available
    if (lc.begin())
    {
      debugMessage(String("Version: 0x")+lc.getICversion(),2);
      lc.setPackAPA(BATTERY_APA);

      hardwareData.batteryPercent = lc.cellPercent();
      hardwareData.batteryVoltage = lc.cellVoltage();
    } 
    else
    {
      // sample battery voltage via analog pin on supported boards
      #if defined (ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
        // modified from the Adafruit power management guide for Adafruit ESP32V2
        float accumulatedVoltage = 0.0;
        for (uint8_t loop = 0; loop < reads; loop++)
        {
          accumulatedVoltage += analogReadMilliVolts(VBATPIN);
        }
         // average the readings
        hardwareData.batteryVoltage = accumulatedVoltage/reads;
        // convert into volts  
        hardwareData.batteryVoltage *= 2;    // we divided by 2, so multiply back
        hardwareData.batteryVoltage /= 1000; // convert to volts!
        hardwareData.batteryVoltage *= 2;     // we divided by 2, so multiply back
        // ESP32 suggested algo
        // hardwareData.batteryVoltage *= 3.3;   // Multiply by 3.3V, our reference voltage
        // hardwareData.batteryVoltage *= 1.05;  // the 1.05 is a fudge factor original author used to align reading with multimeter
        // hardwareData.batteryVoltage /= 4095;  // assumes default ESP32 analogReadResolution (4095)
        hardwareData.batteryPercent = batteryGetChargeLevel(hardwareData.batteryVoltage);
      #endif
    }
    debugMessage(String("Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%",1);
  #endif
}

void batterySimulate()
{
  // IMPROVEMENT: Simulate battery below SCD40 required level
  hardwareData.batteryVoltage = random(batterySimVoltageMin, batterySimVoltageMax) / 100.00;
  hardwareData.batteryPercent = batteryGetChargeLevel(hardwareData.batteryVoltage);
}

int batteryGetChargeLevel(float volts)
// returns battery level as a percentage
{
  uint8_t idx = 50;
  uint8_t prev = 0;
  uint8_t half = 0;
  if (volts >= 4.2)
    return 100;
  if (volts <= 3.2)
    return 0;
  while(true){
    half = abs(idx - prev) / 2;
    prev = idx;
    if(volts >= batteryVoltageTable[idx]){
      idx = idx + half;
    }else{
      idx = idx - half;
    }
    if (prev == idx){
      break;
    }
  }
  return idx;
}

void screenHelperBatteryStatus(uint16_t initialX, uint16_t initialY, uint8_t barWidth, uint8_t barHeight)
// helper function for screenXXX() routines that draws battery charge %
{
  // IMPROVEMENT : Screen dimension boundary checks for passed parameters
  // IMPROVEMENT : Check for offscreen drawing based on passed parameters
  if (hardwareData.batteryVoltage>0.0) 
  {
    // battery nub; width = 3pix, height = 60% of barHeight
    display.fillRect((initialX+barWidth), (initialY+(int(barHeight/5))), 3, (int(barHeight*3/5)), SH110X_WHITE);
    // battery border
    display.drawRect(initialX, initialY, barWidth, barHeight, SH110X_WHITE);
    //battery percentage as rectangle fill, 1 pixel inset from the battery border
    display.fillRect((initialX + 2), (initialY + 2), int(0.5+(hardwareData.batteryPercent*((barWidth-4)/100.0))), (barHeight - 4), SH110X_WHITE);
    debugMessage(String("Battery percent displayed=") + hardwareData.batteryPercent + "%, " + int(0.5+(hardwareData.batteryPercent*((barWidth-4)/100.0))) + " of " + (barWidth-4) + " pixels",1);
  }
  else
    debugMessage("No battery voltage for screenHelperBatteryStatus to render",1);
}

bool sensorCO2Init() {
  #ifdef SENSOR_SIMULATE
    return true;
 #else
    char errorMessage[256];

    // properly initialize boards with two I2C ports
    #if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
      Wire1.begin();
      envSensor.begin(Wire1);
    #else
      // only one I2C port
      Wire.begin();
      envSensor.begin(Wire);
    #endif

    envSensor.wakeUp();
    envSensor.setSensorAltitude(SITE_ALTITUDE);  // optimizes CO2 reading

    uint16_t error = envSensor.startPeriodicMeasurement();
    if (error) {
      // Failed to initialize SCD40
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " executing SCD40 startPeriodicMeasurement()",1);
      return false;
    } 
    else
    {
      debugMessage("power on: SCD40",1);
      return true;
    }
  #endif
}

void sensorCO2Simulate()
// Simulate ranged data from the SCD40
// Improvement - implement stable, rapid rise and fall 
{
  #ifdef SENSOR_SIMULATE
    // Temperature
    // keep this value in C, as it is converted to F in sensorCO2Read
    sensorData.ambientTemperatureF = random(sensorTempMin,sensorTempMax) / 100.0;
    // Humidity
    sensorData.ambientHumidity = random(sensorHumidityMin,sensorHumidityMax) / 100.0;
    // CO2
    sensorData.ambientCO2 = random(sensorCO2Min, sensorCO2Max);
  #endif
}

bool sensorCO2Read()
// reads SCD40 READS_PER_SAMPLE times then stores last read
{
  #ifdef SENSOR_SIMULATE
    sensorCO2Simulate();
  #else
    char errorMessage[256];

    screenAlert(20, ((display.height()/2)+6), "CO2 check");
    for (uint8_t loop=1; loop<=READS_PER_SAMPLE; loop++)
    {
      // SCD40 datasheet suggests 5 second delay between SCD40 reads
      if (loop>1) delay(5000);
      uint16_t error = envSensor.readMeasurement(sensorData.ambientCO2, sensorData.ambientTemperatureF, sensorData.ambientHumidity);
      // handle SCD40 errors
      if (error) {
        errorToString(error, errorMessage, 256);
        debugMessage(String(errorMessage) + " error during SCD4X read",1);
        return false;
      }
      if (sensorData.ambientCO2<400 || sensorData.ambientCO2>6000)
      {
        debugMessage("SCD40 CO2 reading out of range",1);
        return false;
      }
      debugMessage(String("SCD40 read ") + loop + " of " + READS_PER_SAMPLE + " : " + sensorData.ambientTemperatureF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm",2);
    }
  #endif
  //convert temperature from Celcius to Fahrenheit
  sensorData.ambientTemperatureF = (sensorData.ambientTemperatureF * 1.8) + 32;
  #ifdef SENSOR_SIMULATE
      debugMessage(String("SIMULATED SCD40: ") + sensorData.ambientTemperatureF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm",1);
  #else
      debugMessage(String("SCD40: ") + sensorData.ambientTemperatureF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm",1);
  #endif
  return true;
}