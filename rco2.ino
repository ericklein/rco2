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
#include <Adafruit_ST7789.h>
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

#include "Fonts/meteocons16pt7b.h"
// #include <Fonts/FreeSans9pt7b.h>
// #include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans24pt7b.h>

// Special glyphs for the UI
#include "glyphs.h"

#ifndef SENSOR_SIMULATE
  // initialize scd40 CO2 sensor
  #include <SensirionI2CScd4x.h>
  SensirionI2CScd4x envSensor;

  // Battery voltage sensor
  #include "Adafruit_MAX1704X.h"
  Adafruit_MAX17048 lipoBattery;
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

long timeLastSample  = -(sensorSampleInterval*1000);  // set to trigger sample on first iteration of loop()

void setup()
{
  // handle Serial first so debugMessage() works
  #ifdef DEBUG
    Serial.begin(115200);
    // wait for serial port connection
    while (!Serial);
  #endif

  debugMessage(String("RCO2 start with ") + sensorSampleInterval + " second sample interval",1);

  // initialize screen first to display hardware error messages
  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // initialize TFT screen
  display.init(135, 240); // Init ST7789 240x135
  display.setRotation(displayRotation);
  display.setTextWrap(false);
  display.fillScreen(ST77XX_BLACK);

  // Initialize environmental sensor
  if (!sensorCO2Init()) {
    debugMessage("Environment sensor failed to initialize",1);
    screenAlert(5, ((display.height()/2)+6), "No SCD40");
    // This error often occurs right after a firmware flash and reset.
    // Hardware deep sleep typically resolves it, so quickly cycle the hardware
    powerDisable(hardwareRebootInterval);
  }

  // initialize battery monitor
  lipoBattery.setAlertVoltages(batteryVoltageMinAlert,batteryVoltageMaxAlert);
  if (lipoBattery.isHibernating())
    lipoBattery.wake();
}

void loop()
{
  // update current timer value
  unsigned long timeCurrent = millis();

  // is it time to read the sensor?
  if((timeCurrent - timeLastSample) >= (sensorSampleInterval * 1000)) // converting sensorSampleInterval into milliseconds
  {
    // Check battery to see if it is supplying enough voltage to drive the SCD40
    batteryRead();
    if (lipoBattery.isActiveAlert())
    {
      uint8_t status_flags = lipoBattery.getAlertStatus();
      if (status_flags & MAX1704X_ALERTFLAG_VOLTAGE_LOW)
      {
        debugMessage("Battery below required threshold",1);
        screenAlert(20, ((display.height()/2)+6), "Low battery");
        lipoBattery.clearAlertFlag(MAX1704X_ALERTFLAG_VOLTAGE_LOW); // clear the alert
        // freeze user operation. If the device is attached to charger, it will get out of this situation
        // while(1);
      }
    }
    if (!sensorCO2Read())
    {
      debugMessage("SCD40 returned no/bad data",1);
      screenAlert(5, ((display.height()/2)+6),"SCD40 read issue");
      // powerDisable(hardwareRebootInterval);
    }
    screenInfo("");
    timeLastSample = timeCurrent;
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
  // Clear the screen
  display.fillScreen(ST77XX_BLACK);

  display.setTextColor(ST77XX_WHITE);
  display.setFont(&FreeSans24pt7b);
  display.setCursor(initialX, initialY);
  display.print(messageText);
  debugMessage("screenAlert refresh complete",1);
}

void screenInfo(String messageText)
// Display environmental information
{
  debugMessage("screenInfo refresh started",1);
  
  // Clear the screen
  display.fillScreen(ST77XX_BLACK);

  // screen helper routines
  // display battery level in the lower right corner, -3 in first parameter accounts for battery nub
  screenHelperBatteryStatus((display.width()-xMargins-batteryBarWidth-3),(display.height()-yMargins-batteryBarHeight), batteryBarWidth, batteryBarHeight);

  display.setTextColor(ST77XX_WHITE);
  display.setFont(&FreeSans24pt7b);
  display.setCursor(0,30);
  display.println(String("CO2:")+sensorData.ambientCO2);
  display.setFont(&FreeSans18pt7b);
  display.setCursor(0,60);
  display.println(String("Temp:")+sensorData.ambientTemperatureF+"F");
  display.setCursor(0,90);
  display.println(String("Humidity: ")+sensorData.ambientHumidity+"%");

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

void batteryRead()
// sets global battery values from i2c battery monitor or analog pin value on supported boards
{
  #ifdef SENSOR_SIMULATE
    batterySimulate();
    debugMessage(String("SIMULATED Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%",1);
  #else
    // Sample i2c battery monitor
    if (lipoBattery.begin())
    {
      debugMessage(String("Found MAX1704X at I2C address 0x") + lipoBattery.getChipID(),2);
      hardwareData.batteryPercent = lipoBattery.cellPercent();
      hardwareData.batteryVoltage = lipoBattery.cellVoltage();
      delay(2000);
      hardwareData.batteryPercent = lipoBattery.cellPercent();
      hardwareData.batteryVoltage = lipoBattery.cellVoltage();
    } 
    else
    {
      hardwareData.batteryVoltage = 0.0;  // 0.0 means no battery attached
      hardwareData.batteryPercent = 0.0;
    }
    debugMessage(String("Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%",1);
  #endif
}

void batterySimulate()
{
  // IMPROVEMENT: Simulate battery below SCD40 required level
  #ifdef SENSOR_SIMULATE
    hardwareData.batteryVoltage = random(batterySimVoltageMin, batterySimVoltageMax) / 100.00;
    hardwareData.batteryPercent = batteryGetChargeLevel(hardwareData.batteryVoltage);
  #endif
}

void screenHelperBatteryStatus(uint16_t initialX, uint16_t initialY, uint8_t barWidth, uint8_t barHeight)
// helper function for screenXXX() routines that draws battery charge %
{
  // IMPROVEMENT : Screen dimension boundary checks for passed parameters
  // IMPROVEMENT : Check for offscreen drawing based on passed parameters
  if (hardwareData.batteryVoltage>0.0) 
  {
    // battery nub; width = 3pix, height = 60% of barHeight
    display.fillRect((initialX+barWidth), (initialY+(int(barHeight/5))), 3, (int(barHeight*3/5)), ST77XX_WHITE);
    // battery border
    display.drawRect(initialX, initialY, barWidth, barHeight, ST77XX_WHITE);
    //battery percentage as rectangle fill, 1 pixel inset from the battery border
    display.fillRect((initialX + 2), (initialY + 2), int(0.5+(hardwareData.batteryPercent*((barWidth-4)/100.0))), (barHeight - 4), ST77XX_WHITE);
    debugMessage(String("Battery percent displayed is ") + hardwareData.batteryPercent + "%, " + int(0.5+(hardwareData.batteryPercent*((barWidth-4)/100.0))) + " of " + (barWidth-4) + " pixels",1);
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
// reads SCD40 sensorReadsPerSample times then stores last read
{
  #ifdef SENSOR_SIMULATE
    sensorCO2Simulate();
  #else
    char errorMessage[256];

    screenAlert(5, ((display.height()/2)+6), "CO2 check");
    for (uint8_t loop=1; loop<=sensorReadsPerSample; loop++)
    {
      // SCD40 datasheet suggests 5 second delay before SCD40 read
      delay(5000);
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
      debugMessage(String("SCD40 read ") + loop + " of " + sensorReadsPerSample + " : " + sensorData.ambientTemperatureF + "C, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm",2);
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

void powerDisable(uint8_t deepSleepTime)
// turns off component hardware then puts ESP32 into deep sleep mode for specified seconds
{
  debugMessage("powerDisable start",1);
  
  // power down TFT screen
  // turn off backlite
  digitalWrite(TFT_BACKLITE, LOW);

  // turn off the TFT / I2C power supply
  digitalWrite(TFT_I2C_POWER, LOW);
  delay(10);

  //networkDisconnect();

  // power down MAX1704X
  lipoBattery.hibernate();
  debugMessage("power off: MAX1704X",2);

  // power down SCD40 by stopping potentially started measurement then power down SCD40
  #ifndef HARDWARE_SIMULATE
    uint16_t error = envSensor.stopPeriodicMeasurement();
    if (error) {
      char errorMessage[256];
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " executing SCD40 stopPeriodicMeasurement()",1);
    }
    envSensor.powerDown();
    debugMessage("power off: SCD40",2);
  #endif

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    // Turn off the I2C power
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, LOW);
    debugMessage("power off: ESP32V2 I2C",2);
  #endif

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
    // Rev B board is LOW to enable
    // Rev C board is HIGH to enable
    digitalWrite(PIN_I2C_POWER, LOW);
    debugMessage("power off: ESP32S2 I2C",2);
  #endif

  esp_sleep_enable_timer_wakeup(deepSleepTime*1000000); // ESP microsecond modifier
  debugMessage(String("powerDisable complete: ESP32 deep sleep for ") + (deepSleepTime) + " seconds",1);
  esp_deep_sleep_start();
}