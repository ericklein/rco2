/*
  Project:      rco2
  Description:  Regularly sample temperature, humidity, and co2 levels

  See README.md for target information
*/

#include "config.h"   // hardware and internet configuration parameters
#include "secrets.h"  // private credentials
#include "measure.h"  // Utility class for easy handling aggregate sensor data

#ifndef HARDWARE_SIMULATE
  // instanstiate scd40 hardware object
  #include <SensirionI2cScd4x.h>
  SensirionI2cScd4x co2Sensor;

  // ESP32S3 REV TFT has an onboard MAX17048 battery monitor
  #include "Adafruit_MAX1704X.h"
  Adafruit_MAX17048 lipoBattery;
#endif

// screen support (ST7789 240x135 pixels color TFT)
#include <Adafruit_ST7789.h>
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans24pt7b.h>
// Special glyphs for the UI
#include "Fonts/glyphs.h"

// button support
#include <ezButton.h>
ezButton buttonOne(buttonD1Pin,INPUT_PULLDOWN);

// global variables

// environment sensor data
typedef struct {
  // SCD40 data
  float     ambientTemperatureF;
  float     ambientHumidity;      // RH [%]  
  uint16_t  ambientCO2;           // ppm
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
    debugMessage(String("Starting RCO2 with ") + sensorSampleInterval + " second sample interval",1);
  #endif

  #ifdef HARDWARE_SIMULATE
    // generate random numbers for every boot cycle
    // used by HARDWARE_SIUMLATE
    randomSeed(analogRead(0));
  #endif

  // initialize screen first to display hardware error messages

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // initialize TFT screen
  display.init(135, 240); // Init ST7789 240x135
  display.setRotation(screenRotation);
  display.setTextWrap(false);

  screenAlert("Initializing");

  // initialize battery monitor
  batteryInit();

  // Initialize environmental sensor
  if (!sensorCO2Init()) {
    // This error often occurs right after a firmware flash and reset
    debugMessage("Environment sensor failed to initialize",1);
    screenAlert("No SCD40");
    // Hardware deep sleep typically resolves it
    powerDisable(hardwareRebootInterval);
  }
  buttonOne.setDebounceTime(buttonDebounceDelay);
  // Explicit start-up delay because the SCD40 takes ~7 seconds to return valid CO2 readings.
  delay(7000);
}

void loop()
{
  // update current timer value
  unsigned long timeCurrent = millis();

  // Check if battery is supplying enough voltage to drive the SCD40
  // if (lipoBattery.isActiveAlert())
  // {
  //   uint8_t status_flags = lipoBattery.getAlertStatus();
  //   // temp debug code
  //   Serial.print(F("ALERT! flags = 0x"));
  //   Serial.println(status_flags, HEX);
  //   if (status_flags & MAX1704X_ALERTFLAG_VOLTAGE_LOW)
  //   {
  //     debugMessage("Battery below required threshold",1);
  //     screenAlert("Plz charge");
  //     lipoBattery.clearAlertFlag(MAX1704X_ALERTFLAG_VOLTAGE_LOW);
  //     // reboot device. If the device is attached to charger, it will get out of this situation
  //     powerDisable(hardwareRebootInterval);
  //   }
  // } 

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
    if (!sensorCO2Read())
    {
      screenAlert("CO2 read fail");
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

void screenUpdate(bool firstTime) 
{
  switch(screenCurrent) {
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
      screenCurrentData();
      debugMessage("bad screen ID",1);
      break;
  }
}

bool screenAlert(String messageText)
// Description: Display error message centered on screen, using different font sizes and/or splitting to fit on screen
// Parameters: String containing error message text
// Output: NA (void)
// Improvement: ?
{
  bool success = false;
  int16_t x1, y1;
  uint16_t largeFontPhraseOneWidth, largeFontPhraseOneHeight;

  debugMessage("screenAlert start",1);

  display.setTextColor(ST77XX_WHITE);
  display.fillScreen(ST77XX_BLACK);

  debugMessage(String("screenAlert text is '") + messageText + "'",2);

  // does message fit on one line?
  display.setFont(&FreeSans24pt7b);
  display.getTextBounds(messageText.c_str(), 0, 0, &x1, &y1, &largeFontPhraseOneWidth, &largeFontPhraseOneHeight);
  if (largeFontPhraseOneWidth <= (display.width()-(display.width()/2-(largeFontPhraseOneWidth/2)))) {
    // fits with large font, display
    display.setCursor(((display.width()/2)-(largeFontPhraseOneWidth/2)),((display.height()/2)+(largeFontPhraseOneHeight/2)));
    display.print(messageText);
    success = true;
  }
  else {
    // does message fit on two lines?
    debugMessage(String("text with large font is ") + abs(largeFontPhraseOneWidth - (display.width()-(display.width()/2-(largeFontPhraseOneWidth/2)))) + " pixels too long, trying 2 lines", 1);
    // does the string break into two pieces based on a space character?
    uint8_t spaceLocation;
    String messageTextPartOne, messageTextPartTwo;
    uint16_t largeFontPhraseTwoWidth, largeFontPhraseTwoHeight;

    spaceLocation = messageText.indexOf(' ');
    if (spaceLocation) {
      // has a space character, measure two lines
      messageTextPartOne = messageText.substring(0,spaceLocation);
      messageTextPartTwo = messageText.substring(spaceLocation+1);
      display.getTextBounds(messageTextPartOne.c_str(), 0, 0, &x1, &y1, &largeFontPhraseOneWidth, &largeFontPhraseOneHeight);
      display.getTextBounds(messageTextPartTwo.c_str(), 0, 0, &x1, &y1, &largeFontPhraseTwoWidth, &largeFontPhraseTwoHeight);
      debugMessage(String("Message part one with large font is ") + largeFontPhraseOneWidth + " pixels wide",2);
      debugMessage(String("Message part two with large font is ") + largeFontPhraseTwoWidth + " pixels wide",2);
    }
    else {
      debugMessage("there is no space in message to break message into 2 lines",2);
    }
    if (spaceLocation && (largeFontPhraseOneWidth <= (display.width()-(display.width()/2-(largeFontPhraseOneWidth/2)))) && (largeFontPhraseTwoWidth <= (display.width()-(display.width()/2-(largeFontPhraseTwoWidth/2))))) {
        // fits on two lines, display
        display.setCursor(((display.width()/2)-(largeFontPhraseOneWidth/2)),(display.height()/2+largeFontPhraseOneHeight/2)-25);
        display.print(messageTextPartOne);
        display.setCursor(((display.width()/2)-(largeFontPhraseTwoWidth/2)),(display.height()/2+largeFontPhraseTwoHeight/2)+25);
        display.print(messageTextPartTwo);
        success = true;
    }
    else {
      // does message fit on one line with small text?
      debugMessage("couldn't break text into 2 lines or one line is too long, trying small text",1);
      uint16_t smallFontWidth, smallFontHeight;

      display.setFont(&FreeSans18pt7b);
      display.getTextBounds(messageText.c_str(), 0, 0, &x1, &y1, &smallFontWidth, &smallFontHeight);
      if (smallFontWidth <= (display.width()-(display.width()/2-(smallFontWidth/2)))) {
        // fits with small size
        display.setCursor(display.width()/2-smallFontWidth/2,display.height()/2+smallFontHeight/2);
        display.print(messageText);
        success = true;
      }
      else {
        // doesn't fit at any size/line split configuration, display as truncated, large text
        debugMessage(String("text with small font is ") + abs(smallFontWidth - (display.width()-(display.width()/2-(smallFontWidth/2)))) + " pixels too long, displaying truncated", 1);
        display.setFont(&FreeSans12pt7b);
        display.getTextBounds(messageText.c_str(), 0, 0, &x1, &y1, &largeFontPhraseOneWidth, &largeFontPhraseOneHeight);
        display.setCursor(display.width()/2-largeFontPhraseOneWidth/2,display.height()/2+largeFontPhraseOneHeight/2);
        display.print(messageText);
      }
    }
  }
  debugMessage("screenAlert end",1);
  return success;
}

void screenCurrentData()
// Display environmental information
{
  debugMessage("screenCurrentData start",1);

  // screenCurrentData layout assist
  // uint16_t for portability to larger screens
  const uint16_t yCO2 = 40;
  const uint16_t yTempHumidity = 80;
  const uint16_t yIcon = 55;
  const uint16_t xHumidity = 150;
  const uint16_t xIconStep = 50;
  
 // Clear the screen
  display.fillScreen(ST77XX_BLACK);

  // Display CO2 value, highlighting in color based on subjective "goodness"
  display.setTextColor(ST77XX_WHITE);
  display.setFont(&FreeSans24pt7b);
  display.setTextSize(1);
  display.setCursor(xMargins,yCO2);
  display.print(String("CO2: "));
  display.setTextColor(warningColor[co2Range(sensorData.ambientCO2)]);  // Use highlight color look-up table
  display.println(sensorData.ambientCO2,0);
  display.setTextColor(ST77XX_WHITE);

  // Display temperature with symbol from custom glyphs
  display.setFont(&FreeSans18pt7b);
  display.setCursor(xMargins,yTempHumidity);
  display.print(sensorData.ambientTemperatureF,0);
  display.drawBitmap(xMargins + xIconStep,yIcon,epd_bitmap_temperatureF_icon_sm,20,28,0xFFFF);
  
  // Display humidity with symbol from custom glyphs
  display.setCursor(xHumidity,yTempHumidity);
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
  display.fillScreen(warningColor[co2Range(sensorData.ambientCO2)]);  // Use highlight color LUT
  debugMessage("screenColor end",1);
}

void screenSaver()
// Display current CO2 reading at a random location (e.g. "screen saver")
{
  int16_t x, y;

  debugMessage("screenSaver start",1);
  display.fillScreen(ST77XX_BLACK);
  display.setTextSize(1);  // Needed so custom fonts scale properly
  display.setFont(&FreeSans18pt7b);

  // Pick a random location that'll show up
  x = random(xMargins,display.width()-xMargins-64);  // 64 pixels leaves room for 4 digit CO2 value
  y = random(35,display.height()-yMargins); // 35 pixels leaves vertical room for text display
  display.setCursor(x,y);
  display.setTextColor(warningColor[co2Range(sensorData.ambientCO2)]);  // Use highlight color LUT
  display.println(sensorData.ambientCO2);
  debugMessage("screenSaver end",1);
}

void screenAggregateData()
// Displays minimum, average, and maximum values for CO2, temperature and humidity
// using a table-style layout (with labels)
{

  const uint16_t xHeaderColumn = 10;
  const uint16_t xCO2Column = 70;
  const uint16_t xTempColumn = 130;
  const uint16_t xHumidityColumn = 200;
  const uint16_t yHeaderRow = 10;
  const uint16_t yMaxRow = 40;
  const uint16_t yAvgRow = 70;
  const uint16_t yMinRow = 100;

  // clear screen
  display.fillScreen(ST77XX_BLACK);

  // display headers
  display.setFont();  // Revert to built-in font
  display.setTextSize(2);
  display.setTextColor(ST77XX_WHITE);
  // column
  display.setCursor(xCO2Column, yHeaderRow); display.print("CO2");
  display.setCursor(xTempColumn, yHeaderRow); display.print("  F");
  display.setCursor(xHumidityColumn, yHeaderRow); display.print("RH");
  // row
  display.setCursor(xHeaderColumn, yMaxRow); display.print("Max");
  display.setCursor(xHeaderColumn, yAvgRow); display.print("Avg");
  display.setCursor(xHeaderColumn, yMinRow); display.print("Min");

  // Fill in the maximum values row
  display.setCursor(xCO2Column, yMaxRow);
  display.setTextColor(warningColor[co2Range(totalCO2.getMax())]);  // Use highlight color look-up table
  display.print(totalCO2.getMax(),0);
  display.setTextColor(ST77XX_WHITE);
  
  display.setCursor(xTempColumn, yMaxRow); display.print(totalTemperatureF.getMax(),1);
  display.setCursor(xHumidityColumn, yMaxRow); display.print(totalHumidity.getMax(),0);

  // Fill in the average value row
  display.setCursor(xCO2Column, yAvgRow);
  display.setTextColor(warningColor[co2Range(totalCO2.getAverage())]);  // Use highlight color look-up table
  display.print(totalCO2.getAverage(),0);
  display.setTextColor(ST77XX_WHITE);

  display.setCursor(xTempColumn, yAvgRow); display.print(totalTemperatureF.getAverage(),1);
  display.setCursor(xHumidityColumn, yAvgRow); display.print(totalHumidity.getAverage(),0);

  // Fill in the minimum value row
  display.setCursor(xCO2Column,yMinRow);
  display.setTextColor(warningColor[co2Range(totalCO2.getMin())]);  // Use highlight color look-up table
  display.print(totalCO2.getMin(),0);
  display.setTextColor(ST77XX_WHITE);

  display.setCursor(xTempColumn,yMinRow); display.print(totalTemperatureF.getMin(),1);
  display.setCursor(xHumidityColumn,yMinRow); display.print(totalHumidity.getMin(),0);

  // Display current battery level on bottom right of screen
  //screenHelperBatteryStatus((display.width()-xMargins-batteryBarWidth-3),(display.height()-yMargins-batteryBarHeight), batteryBarWidth, batteryBarHeight);
}

void screenGraph()
// Displays CO2 values over time as a graph
{
  // screenGraph specific screen layout assists
  // const uint16_t ySparkline = 95;
  // const uint16_t sparklineHeight = 40;

  debugMessage("screenGraph start",1);
  screenAlert("Graph");
  debugMessage("screenGraph end",1);
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
    // MAX1704X often returns cellPercent > 100 even though cellVoltage < batteryVoltageMax, so constrain the value
    hardwareData.batteryPercent = (hardwareData.batteryPercent > 100.0 ? 100.0 : hardwareData.batteryPercent);
    //battery percentage as rectangle fill, 1 pixel inset from the battery border
    display.fillRect((initialX + 2), (initialY + 2), int(0.5+(hardwareData.batteryPercent*((barWidth-4)/100.0))), (barHeight - 4), ST77XX_WHITE);
    debugMessage(String("Battery percent displayed is ") + hardwareData.batteryPercent + "%, " + int(0.5+(hardwareData.batteryPercent*((barWidth-4)/100.0))) + " of " + (barWidth-4) + " pixels",1);
  }
}

// Hardware simulation routines
#ifdef HARDWARE_SIMULATE
  void batterySimulate()
  // sets global battery values from synthetic algorithms
  {
    // IMPROVEMENT: Simulate battery below SCD40 required level
    hardwareData.batteryVoltage = random(batteryVoltageMin, batteryVoltageMax) / 100.00;
    hardwareData.batteryPercent = ((((hardwareData.batteryVoltage*100.0) - batteryVoltageMin)/(batteryVoltageMax - batteryVoltageMin))*100.0);
    debugMessage(String("SIMULATED Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%",1);
  }

  void sensorSCD4xSimulate(uint8_t mode = 0, uint8_t cycles = 0)
  // Description: Simulates temp, humidity, and CO2 values from Sensirion SCD4X sensor
  // Parameters:
  //  mode
  //    default = random values, ignores cycles parameter
  //    1 = random values, slightly +/- per cycle
  //  cycles = If used, determines how many times the current mode executes before resetting
  // Output : NA
  // Improvement : implement edge value mode, rapid CO2 rise mode 
  {
    static uint8_t currentMode = 1;
    static uint8_t cycleCount = 0;
    static float simulatedTempF;
    static float simulatedHumidity;
    static uint16_t simulatedCO2;

    if (mode != currentMode) {
      cycleCount = 0;
      currentMode = mode;
    }
    switch (currentMode) {
    case 1: // 1 = random values, slightly +/- per cycle
      if (cycleCount == cycles) {
        cycleCount = 0;
      }
      if (!cycleCount) {
        // create new base values
        simulatedTempF = randomFloatRange(sensorTempMinF,sensorTempMaxF);
        simulatedHumidity = randomFloatRange(sensorHumidityMin,sensorHumidityMax);
        simulatedCO2 = random(sensorCO2Min, sensorCO2Max);
        cycleCount++;
      }
      else
      {
        // slightly +/- CO2 value
        int8_t sign = random(0, 2) == 0 ? -1 : 1;
        simulatedCO2 = simulatedCO2 + (sign * random(0, sensorCO2VariabilityRange));
        // slightly +/- temp value
        // slightly +/- humidity value
        cycleCount++;
      }
      break;
    default: // random values, ignores cycles value
      simulatedTempF = randomFloatRange(sensorTempMinF,sensorTempMaxF);
      simulatedHumidity = randomFloatRange(sensorHumidityMin,sensorHumidityMax);
      simulatedCO2 = random(sensorCO2Min, sensorCO2Max);
      break;
    }
    sensorData.ambientTemperatureF = simulatedTempF;
    sensorData.ambientHumidity = simulatedHumidity;
    sensorData.ambientCO2 = simulatedCO2;

    debugMessage(String("Simulated temp: ") + sensorData.ambientTemperatureF + "F, humidity: " + sensorData.ambientHumidity
      + "%, CO2: " + sensorData.ambientCO2 + "ppm",1);
  }
#endif

// Initialize whatever battery monitoring sensor/controller is available
bool batteryInit()
{
  #ifdef HARDWARE_SIMULATE
    return(true);  // Simulating, nothing to be done
  #else
    // Feather ESP32 V2 battery monitoring
    #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
      // Feather ESP32 V2 has no battery sensor/controller, just a simple
      // built-in voltage divider hooked to an analog pin
      return(true); // Nothing to do on ESP32 V2
    #endif
    // Feather ESP32S3 Rev TFT battery monitoring (via onboard MAX17048)
    #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT
      bool status;
      // initialize battery monitor
      status = lipoBattery.begin();
      if(!status) {
        // Couldn't initialize MAX17048, initialization failed
        debugMessage("Failed to initialize MAX17048",1);
        return(false);
      }
      debugMessage(String("MAX1704X battery monitor at I2C address 0x") + lipoBattery.getChipID(),2);
      lipoBattery.setAlertVoltages((batteryVoltageMin/100.0),(batteryVoltageMax/100.0));
      debugMessage(String("Low voltage alert set to ") + (batteryVoltageMin/100.0) + "v",1);
      return(true);
    #endif
  #endif
}

bool batteryRead()
// sets global battery values from i2c battery monitor or analog pin value on supported boards
{
  #ifdef HARDWARE_SIMULATE
    batterySimulate();
    return true;
  #else
    // Feather ESP32 V2 with simple voltage divider battery monitor
    #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
      float measuredvbat = analogReadMilliVolts(VBATPIN);
      measuredvbat *= 2;    // we divided by 2, so multiply back
      measuredvbat /= 1000; // convert to volts!

      if ((measuredvbat*100.0)>=batteryVoltageMin)
      {
        // Estimate battery level by interpolating from known threshold voltages
        hardwareData.batteryVoltage = measuredvbat;
        // Improvement: Could just bit shift measuredvbat below?
        hardwareData.batteryPercent = (((measuredvbat*100.0) - batteryVoltageMin)/(batteryVoltageMax - batteryVoltageMin)/100.0);
        debugMessage(String("Battery voltage from pin read: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%",1);
        return true;
      }
      else
      {
        debugMessage("Low cell voltage, check if battery is connected",1);
        return false;
      }
    #endif
    #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT
      float voltage = lipoBattery.cellVoltage();
      if (isnan(voltage)) {
        debugMessage("Failed to read cell voltage, check if battery is connected",1);
        return false;
      }
      hardwareData.batteryVoltage = voltage;
      hardwareData.batteryPercent = lipoBattery.cellPercent();
      debugMessage(String("Battery voltage from MAX17048: ") + hardwareData.batteryVoltage + "v, " + hardwareData.batteryPercent + "%",1);
      return true;
    #endif
  #endif
}

bool sensorCO2Init()
// initializes SCD40 to read
{
  #ifdef HARDWARE_SIMULATE
    return true;
  #else
    char errorMessage[256];
    uint16_t error;

    Wire.begin();
    co2Sensor.begin(Wire, SCD41_I2C_ADDR_62);

    // stop potentially previously started measurement
    error = co2Sensor.stopPeriodicMeasurement();
    if (error) {
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " executing SCD40 stopPeriodicMeasurement()",1);
      return false;
    }

    // modify configuration settings while not in active measurement mode
    error = co2Sensor.setSensorAltitude(SITE_ALTITUDE);  // optimizes CO2 reading
    if (!error)
      debugMessage(String("SCD40 altitude set to ") + SITE_ALTITUDE + " meters",2);
    else {
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " executing SCD40 setSensorAltitude()",1);
    }

    // Start Measurement.  For high power mode, with a fixed update interval of 5 seconds
    // (the typical usage mode), use startPeriodicMeasurement().  For low power mode, with
    // a longer fixed sample interval of 30 seconds, use startLowPowerPeriodicMeasurement()
    // uint16_t error = co2Sensor.startPeriodicMeasurement();
    error = co2Sensor.startLowPowerPeriodicMeasurement();
    if (error) {
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " executing SCD40 startLowPowerPeriodicMeasurement()",1);
      return false;
    }
    else
    {
      debugMessage("SCD40 starting low power periodic measurements",1);
      return true;
    }
  #endif
}

bool sensorCO2Read()
// Description: Sets global environment values from SCD40 sensor
// Parameters: none
// Output : true if successful read, false if not
// Improvement : NA  
{
  bool success = false;

  #ifdef HARDWARE_SIMULATE
    success = true;
    sensorSCD4xSimulate(1,5);
  #else
    char errorMessage[256];
    uint16_t co2 = 0;
    float temperatureC = 0.0f;
    float humidity = 0.0f;
    uint16_t error;
    uint8_t errorCount = 0;

    // Loop attempting to read Measurement
    debugMessage("CO2 sensor read initiated",1);
    while(!success) {
      delay(100);
      errorCount++;
      if (errorCount > co2SensorReadFailureLimit) {
        debugMessage(String("SCD40 failed to read after ") + errorCount + " attempts",1);
        break;
      }
      // Is data ready to be read?
      bool isDataReady = false;
      error = co2Sensor.getDataReadyStatus(isDataReady);
      if (error) {
          errorToString(error, errorMessage, 256);
          debugMessage(String("Error trying to execute getDataReadyStatus(): ") + errorMessage,1);
          continue; // Back to the top of the loop
      }
      if (!isDataReady) {
          continue; // Back to the top of the loop
      }
      debugMessage("SCD40 data available",2);

      error = co2Sensor.readMeasurement(co2, temperatureC, humidity);
      if (error) {
          errorToString(error, errorMessage, 256);
          debugMessage(String("SCD40 executing readMeasurement(): ") + errorMessage,1);
          // Implicitly continues back to the top of the loop
      }
      else if (co2 < sensorCO2Min || co2 > sensorCO2Max)
      {
        debugMessage(String("SCD40 CO2 reading: ") + co2 + " is out of expected range",1);
        //(sensorData.ambientCO2 < sensorCO2Min) ? sensorData.ambientCO2 = sensorCO2Min : sensorData.ambientCO2 = sensorCO2Max;
        // Implicitly continues back to the top of the loop
      }
      else
      {
        // Valid measurement available, update globals
        sensorData.ambientTemperatureF = (temperatureC*1.8)+32.0;
        sensorData.ambientHumidity = humidity;
        sensorData.ambientCO2 = co2;
        debugMessage(String("SCD40: ") + sensorData.ambientTemperatureF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2 + " ppm",1);
        // Update global sensor readings
        success = true;
        break;
      }
      delay(100); // reduces readMeasurement() "Not enough data received" errors
    }
  #endif
  return(success);
}

uint8_t co2Range(uint16_t co2) 
// converts co2 value to index value for labeling and color
{
  uint8_t co2Range = 
    (co2 <= co2Fair) ? 0 :
    (co2 <= co2Poor) ? 1 :
    (co2 <= co2Bad)  ? 2 : 3;

  debugMessage(String("CO2 input of ") + co2 + " yields co2Range of " + co2Range, 2);
  return co2Range;
}

void powerDisable(uint8_t deepSleepTime)
// turns off component hardware then puts ESP32 into deep sleep mode for specified seconds
{
  debugMessage("powerDisable start",1);

  // power down SCD40 by stopping potentially started measurement then power down SCD40
  #ifndef HARDWARE_SIMULATE
    uint16_t error = co2Sensor.stopPeriodicMeasurement();
    if (error) {
      char errorMessage[256];
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " executing SCD40 stopPeriodicMeasurement()",1);
    }
    co2Sensor.powerDown();
    debugMessage("power off: SCD40",2);
  #endif

    // power down MAX1704
    // Q: is this needed if I already powered down i2c?
    // lipoBattery.hibernate();
    // debugMessage("power off: MAX1704X",2);
  
  //Q: do these two screen related calls work on ESP32v2?
  // power down TFT screen
  // turn off backlite
  digitalWrite(TFT_BACKLITE, LOW);

  // turn off the TFT / I2C power supply
  digitalWrite(TFT_I2C_POWER, LOW);

  // hardware specific powerdown routines
  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    // Turn off the I2C power
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, LOW);
    debugMessage("power off: ESP32V2 I2C",2);
  #endif

  esp_sleep_enable_timer_wakeup(deepSleepTime*1000000); // ESP microsecond modifier
  debugMessage(String("powerDisable complete: ESP32 deep sleep for ") + (deepSleepTime) + " seconds",1);
  esp_deep_sleep_start();
}

float randomFloatRange(uint16_t minVal, uint16_t maxVal) {
  // Scale the range up to hundredths
  uint32_t scaled = random((maxVal - minVal) * 100 + 1);  
  return minVal + (scaled / 100.0f);
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