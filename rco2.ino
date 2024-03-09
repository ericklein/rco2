// Test variant of the RCO2 project to establish proper operation for
// an alternative overeall hardware environment based on the Adafruit ESP32 V2
// Feather, Adafruit 1.14" 240x135 TFT display breakout, along with the same
// SCD40 CO2 sensor.

#include <Arduino.h>
#include <Adafruit_GFX.h>         // Core graphics library
#include <Adafruit_ST7789.h>      // Hardware-specific library for ST7789

// hardware and internet configuration parameters
#include "config.h"

#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans24pt7b.h>

// EZButton library for the onboard button
// ESP32 V2 defines its single button (SW38) input pin as BUTTON
#include <ezButton.h>
ezButton button(buttonPin,BUTTON_MODE);  // Hardcoded for the ESP32S3_REVTFT   DJB: FIX THIS!

// Utility class for easy handling aggregate sensor datta
#include "measure.h"

// Special glyphs for the UI
#include "glyphs.h"

// Sensor support
#ifndef SENSOR_SIMULATE
  // For interacting with the SCD40 sensor
  #include <SensirionI2CScd4x.h>
  #include <Wire.h>
  SensirionI2CScd4x envSensor;

  // ESP32S3 REV TFT has an onboard MAX17048 battery monitor
  #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT
    #include "Adafruit_MAX1704X.h"
    Adafruit_MAX17048 lipoBattery;
  #endif
#endif

// For the 1.14" TFT with ST7789
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

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

// screen layout assists
const uint16_t xMargins = 10;
const uint16_t yMargins = 2;
const uint16_t batteryBarWidth = 28;
const uint16_t batteryBarHeight = 10;

long timeLastSample  = -(sensorSampleInterval*1000);  // set to trigger sample on first iteration of loop()

// Multiple Screen management
#define NUMSCREENS 3  // Total number of supported screens
uint8_t currentScreen = 0;  // Current screen, initialized here to screen #0

void setup() {

  bool status;

  // Need to properly initialize TFT display on ESP32S3 Rev TFT
  #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT

    // turn on backlite
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);

    // turn on the TFT / I2C power supply
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    // delay(10);

  #endif
  
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("Hello! Simple CO2 monitor"));

  button.setDebounceTime(buttonDebounceDelay);

  tft.init(135,240);  // Initialize ST7789 240x135 
  tft.setRotation(displayRotation);
  tft.setTextWrap(false);

  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(0,35);
  tft.setFont(&FreeSans18pt7b);
  tft.println("Initializing...");

  // Initialize SCD40 sensor
  status = sensorCO2Init();
  if(status == false) {
    Serial.println("Error initializing CO2 sensor");
    tft.setCursor(0,75);
    tft.println("NO SCD40!");
  }

  // Initialize battery monitor
  status = batteryInit();
    if(status == false) {
    Serial.println("Error initializing battery");
    tft.setCursor(0,110);
    tft.println("NO BATTERY!");
  }

  Serial.println("Waiting for first measurement... (5 sec)");

}

void loop() {
  
  bool status;
  unsigned long timeCurrent = millis();

  // Must call the button event loop handler first
  button.loop();

  // Now see if a button has been pressed
  if(button.isPressed()) {
    // Move to next screen if button pressed. (Could do on release if preferred)
    currentScreen++;
    if(currentScreen >= NUMSCREENS) currentScreen = 0;
    updateDisplay(true);  // Update to show a new screen (hence true)
  }
  
  if( (timeCurrent - timeLastSample) >= (sensorSampleInterval * 1000) ) {
    // Read the sensor.  This function blocks until valid data is available
    // DJB: sensorCO2Read() currently always returns true (Future enhancement)
    status = sensorCO2Read();
    if(status) {
      // Received new data so update aggregate information
      totalCO2.include(sensorData.ambientCO2);
      totalTemperatureF.include(sensorData.ambientTemperatureF);
      totalHumidity.include(sensorData.ambientHumidity);
    }

    // Get battery status info
    status = batteryRead();

    // Update the TFT display with new readings on the current screen (hence false)
    updateDisplay(false);

    totalCO2.printMeasure();
    totalTemperatureF.printMeasure();

    // Save current timestamp to restart sample delay interval
    timeLastSample = timeCurrent;
  }
}


// Initialize SCD40 -- called once in setup().  Closely follows code from
// Sensirion example sketch to properly manage data access and read operations.
bool sensorCO2Init()
{
  #ifdef SENSOR_SIMULATE
    delay(2000);   // Not unlike actual sensor & I2C initialization time
    return(true);  // Simulation always works (and has nothing to init)
  #else
    // Not simulating, so initialize the SCD40
    uint16_t error;
    char errorMessage[256];
    bool status = true;

    Wire.begin();
    envSensor.begin(Wire);

    // stop potentially previously started measurement.
    error = envSensor.stopPeriodicMeasurement();
    if (error) {
      Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
      status = false;
    }

    // Check onboard configuration settings while not in active measurement mode
    float offset;
    uint16_t sensor_altitude;
    error = envSensor.getTemperatureOffset(offset);
    Serial.print("SCD40 Temperature offset: ");
    Serial.println(offset,2);
    error = envSensor.getSensorAltitude(sensor_altitude);
    Serial.print("SCD40 Altitude: ");
    Serial.println(sensor_altitude);
    error = envSensor.setTemperatureOffset(0);
    error = envSensor.setSensorAltitude(400);

    // Start Measurement.  For high power mode, with a fixed update interval of 5 seconds
    // (the typical usage mode), use startPeriodicMeasurement().  For low power mode, with
    // a longer fixed sample interval of 30 seconds, use startLowPowerPeriodicMeasurement()
    error = envSensor.startLowPowerPeriodicMeasurement();
    if (error) {
      Serial.print("Error trying to execute startLowPowerPeriodicMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
      status = false;
    }
    return(status);
  #endif
}

// Read the SCD40 sensor.  This function blocks until readings are
// ready and return valid values.
//
// DJB: Enhancement = non-blocking version that returns data status
bool sensorCO2Read()
{
  // For simulation, generate semi-random data and return
  #ifdef SENSOR_SIMULATE
    sensorCO2Simulate();  // Generate quasi-random values in lieu of actual sensor data
    return(true);
  #else
    // Not simulating, so read the SCD40
    uint16_t error;
    char errorMessage[256];
    bool status;
    uint16_t co2 = 0;
    float temperature = 0.0f;
    float humidity = 0.0f;

    Serial.println("Initiating CO2 read");
    // Loop attempting to read Measurement
    status = false;
    while(!status) {
      delay(100);

      // Is data ready to be read?
      bool isDataReady = false;
      error = envSensor.getDataReadyFlag(isDataReady);
      if (error) {
          Serial.print("Error trying to execute getDataReadyFlag(): ");
          errorToString(error, errorMessage, 256);
          Serial.println(errorMessage);
          continue; // Back to the top of the loop
      }
      if (!isDataReady) {
          continue; // Back to the top of the loop
      }
      Serial.println("Sensor data ready!");
      error = envSensor.readMeasurement(co2, temperature, humidity);
      if (error) {
          Serial.print("Error trying to execute readMeasurement(): ");
          errorToString(error, errorMessage, 256);
          Serial.println(errorMessage);
          // Implicitly continues back to the top of the loop
      } else if (co2 == 0) {
          Serial.println("Invalid sample detected, skipping.");
          // Implicitly continues back to the top of the loop
      } else {
          // Successfully read valid data!
          Serial.print("Co2:");
          Serial.print(co2);
          Serial.print("\t");
          Serial.print("Temperature:");
          Serial.print((temperature*1.8)+32.0);
          Serial.print("\t");
          Serial.print("Humidity:");
          Serial.println(humidity);

          // Update global sensor readings
          sensorData.ambientTemperatureF = (temperature*1.8)+32.0;
          sensorData.ambientHumidity = humidity;
          sensorData.ambientCO2 = co2;
          status = true;  // We have data, can break out of loop
      }
    }
    // If we get here we read valid data and can return successfull (true)
    return(true);
  #endif
}

void sensorCO2Simulate()
// sets global environment values from synthetic algorithms
{
  // Improvement - implement stable, rapid rise and fall
  #ifdef SENSOR_SIMULATE
    // Temperature
    // SCD40 reports in Celsius so we simulate that and then convert
    sensorData.ambientTemperatureF = 32.0 + 1.8*(random(sensorTempMin,sensorTempMax) / 100.0);
    // Humidity
    sensorData.ambientHumidity = random(sensorHumidityMin,sensorHumidityMax) / 100.0;
    // CO2
    sensorData.ambientCO2 = random(sensorCO2Min, sensorCO2Max);
  #endif
}

// Initialize whatever battery monitoring sensor/controller may be
// available.
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
{
  #ifdef SENSOR_SIMULATE
    batterySimulate();
    return(true);
  #else
    Serial.println("Reading battery");
    // Feather ESP32 V2 with simple voltage divider battery monitor
    #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
      float measuredvbat = analogReadMilliVolts(VBATPIN);
      measuredvbat *= 2;    // we divided by 2, so multiply back
      measuredvbat /= 1000; // convert to volts!

      // Estimate battery level by interpolating from known threshold voltages
      float vmin = batteryVoltageMinAlert;
      float vmax = batteryVoltageMaxAlert;
      hardwareData.batteryPercent = 100.0*(measuredvbat - vmin)/(vmax - vmin);
      hardwareData.batteryVoltage = measuredvbat;
      return(true);
    #endif

    // Feather ESP32S3 with Reverse TFT, which manages the battery via an
    // onboard MAX17048 controller
    #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT
      hardwareData.batteryPercent = lipoBattery.cellPercent();
      hardwareData.batteryVoltage = lipoBattery.cellVoltage();
      return(true);
    #endif

  #endif
}

// Simulate battery values, both voltage and percentage of capacity
void batterySimulate()
{
  // IMPROVEMENT: Simulate battery below SCD40 required level
  #ifdef SENSOR_SIMULATE
    float vbat100 = random(batterySimVoltageMin, batterySimVoltageMax);
    hardwareData.batteryVoltage = vbat100 / 100.0;
    hardwareData.batteryPercent = 100.0 * (vbat100 - batterySimVoltageMin) / (batterySimVoltageMax - batterySimVoltageMin);
  #endif
}

// Updates TFT display
void displayCurrentData(bool firstTime)
{
  // Display current CO2 reading
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);  // Needed so custom fonts scale properly
  tft.setFont(&FreeSans24pt7b);
  tft.setCursor(0,35);
  tft.print("CO2: ");
  uint8_t co2range = ((sensorData.ambientCO2 - 400) / 400);
  co2range = constrain(co2range,0,4); // filter CO2 levels above 2400
  tft.setTextColor(co2Highlight[co2range]);  // Use highlight color look-up table
  tft.println(sensorData.ambientCO2);
  tft.setTextColor(ST77XX_WHITE);

  // Display current temperature and humidity together on a single line
  tft.setFont(&FreeSans18pt7b);
  tft.setCursor(0,75);
  tft.print(sensorData.ambientTemperatureF,1);
  tft.drawBitmap(75,51,epd_bitmap_temperatureF_icon_sm,20,28,0xFFFF);

  tft.setCursor(150,75);
  tft.print(sensorData.ambientHumidity,0);
  tft.drawBitmap(195,50,epd_bitmap_humidity_icon_sm4,20,28,0xFFFF);

  // Display current battery level on bottom right of screen
  screenHelperBatteryStatus((tft.width()-xMargins-batteryBarWidth-3),(tft.height()-yMargins-batteryBarHeight), batteryBarWidth, batteryBarHeight);

}

// Displays minimum, average, and maximum values for CO2, temperature and humidity
// using a table-style layout (with labels)
void displayAggregateData(bool firstTime)
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

void screenHelperBatteryStatus(uint16_t initialX, uint16_t initialY, uint8_t barWidth, uint8_t barHeight)
// helper function for screenXXX() routines that draws battery charge %
{
  // IMPROVEMENT : Screen dimension boundary checks for passed parameters
  // IMPROVEMENT : Check for offscreen drawing based on passed parameters
  if (hardwareData.batteryVoltage>0.0) 
  {
    // battery nub; width = 3pix, height = 60% of barHeight
    tft.fillRect((initialX+barWidth), (initialY+(int(barHeight/5))), 3, (int(barHeight*3/5)), ST77XX_WHITE);
    // battery border
    tft.drawRect(initialX, initialY, barWidth, barHeight, ST77XX_WHITE);
    //battery percentage as rectangle fill, 1 pixel inset from the battery border
    tft.fillRect((initialX + 2), (initialY + 2), int(0.5+(hardwareData.batteryPercent*((barWidth-4)/100.0))), (barHeight - 4), ST77XX_WHITE);
    /*
    Serial.println(String("Battery percent displayed is ") + hardwareData.batteryPercent + "%, " + int(0.5+(hardwareData.batteryPercent*((barWidth-4)/100.0))) + " of " + (barWidth-4) + " pixels");
    */
  }
  else
    Serial.println("No battery voltage for screenHelperBatteryStatus to render");
}

void updateDisplay(bool firstTime) 
{
  switch(currentScreen) {
    case 0:
      displayCurrentData(firstTime);
      break;
    case 1:
      displayAggregateData(firstTime);
      break;
    case 2:
      screenSaver(firstTime);
      break;
    default:
      // This shouldn't happen, but if it does...
      displayCurrentData(firstTime);
      Serial.println("BAD SCREEN ID!");
      break;
  }
}

void screenSaver(bool firstTime)
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
