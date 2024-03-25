/*
  Project:      rco2
  Description:  Regularly sample temperature, humidity, and co2 levels

  See README.md for target information
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT
#include "secrets.h"

// Utility class for easy handling aggregate sensor datta
#include "measure.h"

// screen support (ST7789 240x135 pixels color TFT)
#include <Adafruit_ST7789.h>
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans24pt7b.h>
// Special glyphs for the UI
#include "glyphs.h"

// sensor support
#ifndef SENSOR_SIMULATE
  // initialize scd40 CO2 sensor
  #include <SensirionI2CScd4x.h>
  SensirionI2CScd4x envSensor;
  // ESP32S3 REV TFT has an onboard MAX17048 battery monitor
  #include "Adafruit_MAX1704X.h"
  Adafruit_MAX17048 lipoBattery;
#endif

// button support
#include <ezButton.h>
ezButton buttonOne(buttonD1Pin,INPUT_PULLDOWN);

// global variables

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
  //uint8_t rssi;
} hdweData;
hdweData hardwareData;

// Utility class used to streamline accumulating sensor values
Measure totalCO2, totalTemperatureF, totalHumidity;

long timeLastSample  = -(sensorSampleInterval*1000);  // set to trigger sample on first iteration of loop()
uint8_t screenCurrent = 0;

void setup()
{
  // handle Serial first so debugMessage() works
  #ifdef DEBUG
    Serial.begin(115200);
    // wait for serial port connection
    while (!Serial);

    debugMessage(String("RCO2 start with ") + sensorSampleInterval + " second sample interval",1);
  #endif

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
  display.setRotation(screenRotation);
  display.setTextWrap(false);

  screenAlert("Initializing");

  // initialize battery monitor if available
  if (batteryInit())
  {
    lipoBattery.setAlertVoltages(batteryVoltageMinAlert,batteryVoltageMaxAlert);
    if (lipoBattery.isHibernating())
      lipoBattery.wake(); 
    // Check if battery is supplying enough voltage to drive the SCD40
    if (lipoBattery.isActiveAlert())
    {
      uint8_t status_flags = lipoBattery.getAlertStatus();
      if (status_flags & MAX1704X_ALERTFLAG_VOLTAGE_LOW)
      {
        debugMessage("Battery below required threshold",1);
        screenAlert("Charge device");
        lipoBattery.clearAlertFlag(MAX1704X_ALERTFLAG_VOLTAGE_LOW); // clear the alert
        // stop user operation. If the device is attached to charger, it will get out of this situation
        while(1);
      }
    }   
  }

  // Initialize environmental sensor
  if (!sensorCO2Init()) {
    debugMessage("Environment sensor failed to initialize",1);
    screenAlert("No CO2 sensor");
    // This error often occurs right after a firmware flash and reset.
    // Hardware deep sleep typically resolves it, so quickly cycle the hardware
    powerDisable(hardwareRebootInterval);
  }

  buttonOne.setDebounceTime(buttonDebounceDelay); 
}

void loop()
{
  // update current timer value
  unsigned long timeCurrent = millis();

  buttonOne.loop();
  // check if buttons were pressed
  if (buttonOne.isReleased())
  {
    ((screenCurrent + 1) >= screenCount) ? screenCurrent = 0 : screenCurrent ++;
    debugMessage(String("button 1 press, switch to screen ") + screenCurrent,1);
    screenUpdate(true);
  }

  // is it time to read the sensor?
  if((timeCurrent - timeLastSample) >= (sensorSampleInterval * 1000)) // converting sensorSampleInterval into milliseconds
  {
    // Check if battery is supplying enough voltage to drive the SCD40
    if (lipoBattery.isActiveAlert())
    {
      uint8_t status_flags = lipoBattery.getAlertStatus();
      if (status_flags & MAX1704X_ALERTFLAG_VOLTAGE_LOW)
      {
        debugMessage("Battery below required threshold",1);
        screenAlert("Charge device");
        lipoBattery.clearAlertFlag(MAX1704X_ALERTFLAG_VOLTAGE_LOW); // clear the alert
        // stop user operation. If the device is attached to charger, it will get out of this situation
        while(1);
      }
    }

    if (!sensorCO2Read())
    {
      screenAlert("SCD40 bad read");
      // powerDisable(hardwareRebootInterval);
    }
    else
    {
      // Received new data so update aggregate information
      totalCO2.include(sensorData.ambientCO2);
      totalTemperatureF.include(sensorData.ambientTemperatureF);
      totalHumidity.include(sensorData.ambientHumidity);// refresh current screen based on new sensor reading
      // Update the TFT display with new readings on the current screen (hence false)
      screenUpdate(true);
    }
    // Save current timestamp to restart sample delay interval
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

void screenUpdate(bool firstTime) 
{
  switch(currentScreen) {
    case 0:
      screenSaver();
      break;
    case 1:
      screenCurrentData();
      break;
    case 2:
      screenAggregateData();
      break;
    case 3:
      screenColor();
      break;
    case 4:
      screenGraph();
      break;
    default:
      // This shouldn't happen, but if it does...
      displayCurrentData();
      debugMessage("bad screen ID",1);
      break;
  }
}

void screenAlert(String messageText)
// Display error message centered on screen
{
  debugMessage("screenAlert start",1);
  // Clear the screen
  display.fillScreen(ST77XX_BLACK);

  int16_t x1, y1;
  uint16_t width, height;

  display.setTextColor(ST77XX_WHITE);
  display.setFont(&FreeSans24pt7b);
  display.getTextBounds(messageText.c_str(), 0, 0, &x1, &y1, &width, &height);
  if (width >= display.width()) {
    debugMessage(String("ERROR: screenAlert '") + messageText + "' is " + abs(display.width()-width) + " pixels too long", 1);
  }
  // test: clear a band around the text to be displayed
  // display.fillRect(((display.width() / 2) - ((width / 2) + 10)), ((display.height() / 2) - ((height / 2) + 3)), width + 20, height + 10, ST77XX_BLACK);
  // display text
  display.setCursor(display.width() / 2 - width / 2, display.height() / 2 + height / 2);
  display.print(messageText);
  debugMessage("screenAlert end",1);
}

void screenCurrentData()
// Display environmental information
{
  debugMessage("screenCurrentData start",1);

  // screenCurrentData layout assist
  // uint16_t for portability to larger screens
  const uint16_t yCO2 = 35;
  const uint16_t yTempHumidity = 75;
  const uint16_t yIcon = 50;
  const uint16_t xHumidity = 155;
  const uint16_t xIconStep = 50;
  
 // Clear the screen
  display.fillScreen(ST77XX_BLACK);

  // Display CO2 value, highlighting in color based on subjective "goodness"
  display.setTextColor(ST77XX_WHITE);
  display.setFont(&FreeSans24pt7b);
  display.setTextSize(1);
  display.setCursor(xMargins,yCO2);
  display.print(String("CO2: "));
  uint8_t co2range = ((sensorData.ambientCO2 - 400) / 400);
  co2range = constrain(co2range,0,4); // filter CO2 levels above 2400
  display.setTextColor(co2Highlight[co2range]);  // Use highlight color look-up table
  display.println(sensorData.ambientCO2,0);
  
  // Display temperature with symbol from custom glyphs
  display.setFont(&FreeSans18pt7b);
  display.setCursor(xMargins,yTempHumidity);
  display.print(sensorData.ambientTemperatureF,0);
  display.drawBitmap(xMargins + xIconStep,yIcon,epd_bitmap_temperatureF_icon_sm,20,28,0xFFFF);
  
  // Display humidity with symbol from custom glyphs
  display.setCursor(150,yTempHumidity);
  display.print(sensorData.ambientHumidity,0); 
  display.drawBitmap(xHumidity + xIconStep,yIcon,epd_bitmap_humidity_icon_sm4,20,28,0xFFFF);

  // display battery level in the lower right corner, -3 in first parameter accounts for battery nub
  screenHelperBatteryStatus((display.width()-xMargins-batteryBarWidth-3),(display.height()-yMargins-batteryBarHeight), batteryBarWidth, batteryBarHeight);

  debugMessage("screenCurrentData end",1);
}

void screenColor()
// Represents CO2 value on screen as a single color fill
{
  debugMessage("screenColor start",1);
  uint8_t co2range = ((sensorData.ambientCO2 - 400) / 400);
  co2range = constrain(co2range,0,4); // filter CO2 levels above 2400
  display.fillScreen(co2Highlight[co2range]);  // Use highlight color look-up table
   // screen helper routines
  // display battery level in the lower right corner, -3 in first parameter accounts for battery nub
  screenHelperBatteryStatus((display.width()-xMargins-batteryBarWidth-3),(display.height()-yMargins-batteryBarHeight), batteryBarWidth, batteryBarHeight);
  debugMessage("screenColor end",1);
}

void screenSaver()
{
  int16_t x, y;

  // Display current CO2 reading at a random location ("screen saver")
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);  // Needed so custom fonts scale properly
  tft.setFont(&FreeSans18pt7b);

  // Pick a random location that'll show up
  x = random(xMargins,240-xMargins-64);  // Guessing 64 is room for 4 digit CO2 value
  y = random(35,135-yMargins);
  tft.setCursor(x,y);

  uint8_t co2range = ((sensorData.ambientCO2 - 400) / 400);
  co2range = constrain(co2range,0,4); // filter CO2 levels above 2400
  tft.setTextColor(co2Highlight[co2range]);  // Use highlight color look-up table
  tft.println(sensorData.ambientCO2);
  tft.setTextColor(ST77XX_WHITE);
}

// Displays minimum, average, and maximum values for CO2, temperature and humidity
// using a table-style layout (with labels)
void screenAggregateData()
{  
  uint8_t co2range;

  // Clear screen and display row/column labels
  tft.fillScreen(ST77XX_BLACK);
  tft.setFont();  // Revert to built-in font
  tft.setTextSize(2);

  tft.setCursor( 70, 10); tft.print("CO2");
  tft.setCursor(130, 10); tft.print("  F");
  tft.setCursor(200, 10); tft.print("RH");

  tft.setCursor( 10, 40); tft.print("Max");
  tft.setCursor( 10, 70); tft.print("Avg");
  tft.setCursor( 10,100); tft.print("Min");

  // Fill in the maximum values row
  tft.setCursor( 70, 40);
  co2range = ((totalCO2.getMax() - 400) / 400);
  co2range = constrain(co2range,0,4); // filter CO2 levels above 2400
  tft.setTextColor(co2Highlight[co2range]);  // Use highlight color look-up table
  tft.print(totalCO2.getMax(),0);
  tft.setTextColor(ST77XX_WHITE);
  
  tft.setCursor(130, 40); tft.print(totalTemperatureF.getMax(),1);
  tft.setCursor(200, 40); tft.print(totalHumidity.getMax(),0);

  // Fill in the average value row
  tft.setCursor( 70, 70);
  co2range = ((totalCO2.getAverage() - 400) / 400);
  co2range = constrain(co2range,0,4); // filter CO2 levels above 2400
  tft.setTextColor(co2Highlight[co2range]);  // Use highlight color look-up table
  tft.print(totalCO2.getAverage(),0);
  tft.setTextColor(ST77XX_WHITE);

  tft.setCursor(130, 70); tft.print(totalTemperatureF.getAverage(),1);
  tft.setCursor(200, 70); tft.print(totalHumidity.getAverage(),0);

  // Fill in the minimum value row
  tft.setCursor( 70,100);   
  co2range = ((totalCO2.getMin() - 400) / 400);
  co2range = constrain(co2range,0,4); // filter CO2 levels above 2400
  tft.setTextColor(co2Highlight[co2range]);  // Use highlight color look-up table
  tft.print(totalCO2.getMin(),0);
  tft.setTextColor(ST77XX_WHITE);

  tft.setCursor(130,100); tft.print(totalTemperatureF.getMin(),1);
  tft.setCursor(200,100); tft.print(totalHumidity.getMin(),0);


  // Display current battery level on bottom right of screen
  screenHelperBatteryStatus((tft.width()-xMargins-batteryBarWidth-3),(tft.height()-yMargins-batteryBarHeight), batteryBarWidth, batteryBarHeight);
}

void screenGraph()
// Displays CO2 values over time as a graph
{
  // screenGraph specific screen layout assists
  // const uint16_t ySparkline = 95;
  // const uint16_t sparklineHeight = 40;

  debugMessage("screenGraph start",1);
  screenAlert("screenGraph");
  debugMessage("screenGraph end",1);
}

// Initialize whatever battery monitoring sensor/controller is available
bool batteryInit()
{
  #ifdef SENSOR_SIMULATE
    return(true);  // Simulating, nothing to be done
  #else
    bool status;

    // Feather ESP32 V2 battery monitoring
    #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
      // Feather ESP32 V2 has no battery sensor/controller, just a simple
      // built-in voltage divider hooked to an analog pin
      return(true); // Nothing to do on ESP32 V2
    #endif

    // Feather ESP32S3 Rev TFT battery monitoring (via onboard MAX17048)
    #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT
      // initialize battery monitor
      status = lipoBattery.begin();
      if(!status) {
        // Couldn't initialize MAX17048, initialization failed
        Serial.println("Failed to initialize MAX17048 battery manager!");
        return(false);
      }
      Serial.println(String("Found MAX1704X at I2C address 0x") + lipoBattery.getChipID());
      if (lipoBattery.isHibernating()) lipoBattery.wake();
      lipoBattery.setAlertVoltages(batteryVoltageMinAlert,batteryVoltageMaxAlert);
      return(true);
    #endif
  #endif
}

bool batteryRead()
// sets global battery values from i2c battery monitor or analog pin value on supported boards
{
  #ifdef SENSOR_SIMULATE
    batterySimulate();
    debugMessage(String("SIMULATED Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%",1);
    return true;
  #else
    // Feather ESP32 V2 with simple voltage divider battery monitor
    #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
      float measuredvbat = analogReadMilliVolts(VBATPIN);
      measuredvbat *= 2;    // we divided by 2, so multiply back
      measuredvbat /= 1000; // convert to volts!

      // Estimate battery level by interpolating from known threshold voltages
      hardwareData.batteryPercent = 100.0*(measuredvbat - batteryVoltageMinAlert)/(batteryVoltageMaxAlert - batteryVoltageMinAlert);
      hardwareData.batteryVoltage = measuredvbat;
      if (hardwareData.batteryVoltage>=batteryVoltageMinAlert)
      {
        debugMessage(String("Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%",1);
        return true;
      }
      else
      {
        debugMessage("No battery?",1);
        return false;
      }
    #endif
    #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT
      if (lipoBattery.begin())
      {
        debugMessage(String("Found MAX1704X at I2C address 0x") + lipoBattery.getChipID(),2);
        hardwareData.batteryPercent = lipoBattery.cellPercent();
        hardwareData.batteryVoltage = lipoBattery.cellVoltage();
        // documentation called for a second read to handle error condition?
        //delay(2000);
        // hardwareData.batteryPercent = lipoBattery.cellPercent();
        // hardwareData.batteryVoltage = lipoBattery.cellVoltage();
        debugMessage(String("Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%",1);
        return true;
      } 
      else
        return false;
    #endif
  #endif
}

void batterySimulate()
// sets global battery values from synthetic algorithms
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
 
  if (batteryRead())
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
    debugMessage("No battery for screenHelperBatteryStatus to render",1);
}

bool sensorCO2Init()
// initializes CO2 sensor to read
{
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
// sets global environment values from synthetic algorithms
{
  // Improvement - implement stable, rapid rise and fall
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
// sets global environment values from SCD40 sensor
{
  debugMessage("sensorCO2 read start",2);
  #ifdef SENSOR_SIMULATE
    sensorCO2Simulate();
  #else
    char errorMessage[256];

    for (uint8_t loop=1; loop<=sensorReadsPerSample; loop++)
    {
      // SCD40 datasheet suggests 5 second delay before SCD40 read
      delay(5000);
      uint16_t error = envSensor.readMeasurement(sensorData.ambientCO2, sensorData.ambientTemperatureF, sensorData.ambientHumidity);
      //convert temperature from Celcius to Fahrenheit
      sensorData.ambientTemperatureF = (sensorData.ambientTemperatureF * 1.8) + 32;
      // handle SCD40 errors
      if (error) {
        errorToString(error, errorMessage, 256);
        debugMessage(String("SCD40 read error: ") + errorMessage,1);
        return false;
      }
      if (sensorData.ambientCO2<400 || sensorData.ambientCO2>6000)
      {
        debugMessage(String("SCD40 CO2 read out of range: ") + sensorData.ambientCO2,1);
        (sensorData.ambientCO2<400) ? sensorData.ambientCO2 = 400 : sensorData.ambientCO2 = 6000;
        return false;
      }
      debugMessage(String("SCD40 read ") + loop + " of " + sensorReadsPerSample + " : " + sensorData.ambientTemperatureF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm",2);
    }
  #endif
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

  // power down MAX1704
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