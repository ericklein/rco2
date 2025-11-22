/*
  Project:      rco2
  Description:  Regularly sample temperature, humidity, and co2 levels

  See README.md for target information
*/

#include "config.h"   // hardware and internet configuration parameters
#include "secrets.h"  // private credentials
#include "measure.h"  // Utility class for easy handling aggregate sensor data

#include "driver/rtc_io.h"

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
#include <Fonts/FreeSans9pt7b.h>
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
  float     ambientHumidity;            // RH [%]  
  int16_t  ambientCO2[co2GraphPoints];  // ppm, ppm, range 400 to 2000, -1 = no data
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

int32_t timeLastSampleMS  = -(sensorSampleIntervalMS);  // set to trigger sample on first iteration of loop()
uint32_t timeLastSleepMS = 0;
uint8_t screenCurrent = 0;

void setup()
{
  // handle Serial first so debugMessage() works
  #ifdef DEBUG
    Serial.begin(115200);
    // wait for serial port connection
    while (!Serial);
    debugMessage(String("Starting RCO2 with ") + (sensorSampleIntervalMS/1000) + " second sample interval",1);
  #endif

  #ifdef HARDWARE_SIMULATE
    // generate random numbers for every boot cycle
    // used by HARDWARE_SIUMLATE
    randomSeed(analogRead(0));
  #endif

  // Set ESP32 external trigger ext0 for one button wakeup interupt
  rtc_gpio_pullup_dis(WAKE_FROM_SLEEP_PIN);
  rtc_gpio_pulldown_en(WAKE_FROM_SLEEP_PIN);

  esp_err_t result = esp_sleep_enable_ext0_wakeup(WAKE_FROM_SLEEP_PIN,1);  //1 = High, 0 = Low
  if (result == ESP_OK) {
    debugMessage("EXT0 Wake-Up set successfully as wake-up source.",1);
  } 
  else {
    debugMessage("Failed to set EXT0 Wake-Up as wake-up source.",1);
  }

  // Set ESP32 light sleep interval
  result = esp_sleep_enable_timer_wakeup(hardwareLightSleepTimeμS);
  // delay(100);
  if (result == ESP_OK) {
    Serial.println("Timer Wake-Up set successfully as wake-up source.");
  } else {
      Serial.println("Failed to set Timer Wake-Up as wake-up source.");
  }

  // initialize CO2 array for graphing
  for(uint8_t loop=0;loop<co2GraphPoints;loop++) {
    sensorData.ambientCO2[loop] = -1;
  }

  // initialize battery monitor
  batteryInit();

  // initialize screen first to display hardware error messages

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // initialize TFT screen
  display.init(135, 240); // Init ST7789 240x135
  display.setRotation(screenRotation);
  display.setTextWrap(false);

  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  screenAlert("Initializing");

  buttonOne.setDebounceTime(buttonDebounceDelayMS);

  // Initialize environmental sensor
  if (!sensorSCD4xSSInit()) {
    // This error often occurs right after a firmware flash and reset
    debugMessage("Environment sensor failed to initialize",1);
    screenAlert("No SCD40");
    // Hardware deep sleep typically resolves it
    powerDeepSleep(hardwareErrorSleepTimeμS);
  }
  #ifndef HARDWARE_SIMULATE
    // Explicit start-up delay because the SCD40 takes ~7 seconds to return valid CO2 readings.
    // ? needed with single shot?
    delay(7000);
    timeLastSleepMS = millis(); // compensate for the long delay()
  #endif
}

// new loop()

// check battery
// timing based single shot read and update current screen
// check button and update screen if needed
// display screen for x seconds
// go to sleep (screen, sensor, esp32) for x seconds or GPIO wakeup

void loop()
{
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
  //     powerDeepSleep(hardwareRebootInterval);
  //   }
  // } 

  buttonOne.loop();
  // check if buttons were pressed
  if (buttonOne.isReleased())
  {
    ((screenCurrent + 1) >= screenCount) ? screenCurrent = 0 : screenCurrent ++;
    debugMessage(String("button press, switch to screen ") + screenCurrent,1);
    screenUpdate(true);
  }

  // is it time to read the sensor?
  if((millis() - timeLastSampleMS) >= (sensorSampleIntervalMS))
  {
    if (!sensorSCD4xSSRead())
    {
      screenAlert("CO2 read fail");
    }
    else {
      // Received new data so update aggregate information
      totalCO2.include(sensorData.ambientCO2[co2GraphPoints-1]);
      totalTemperatureF.include(sensorData.ambientTemperatureF);
      totalHumidity.include(sensorData.ambientHumidity);// refresh current screen based on new sensor reading
      // Update the TFT display with new readings on the current screen (hence false)
      screenUpdate(true);
    }
    // Save current timestamp to restart sample delay interval
    timeLastSampleMS = millis();
  }

  // is it time to sleep and wakeup?
  if((millis() - timeLastSleepMS) >= (screenDisplayTimeMS)) {
    powerLightSleep(hardwareLightSleepTimeμS);
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason)
    {
      case ESP_SLEEP_WAKEUP_TIMER : // do nothing
      {
        debugMessage("wakeup cause: timer",1);
      }
      break;
      case ESP_SLEEP_WAKEUP_EXT0 :
      {
        debugMessage("wakeup cause: RTC gpio pin",1);
        delay(500);  // Debounce?
      }
      break;
      // case ESP_SLEEP_WAKEUP_EXT1 :
      // {
      //   uint16_t gpioReason = log(esp_sleep_get_ext1_wakeup_status())/log(2);
      //   debugMessage(String("wakeup cause: RTC gpio pin: ") + gpioReason,1);
      //   // implment switch (gpioReason)
      // }
      // break;
      // case ESP_SLEEP_WAKEUP_TOUCHPAD : 
      // {
      //   debugMessage("wakup cause: touchpad",1);
      // }  
      // break;
      // case ESP_SLEEP_WAKEUP_ULP : 
      // {
      //   debugMessage("wakeup cause: program",1);
      // }  
      // break; 
      default :
      {
        // likely caused by reset after firmware load
        debugMessage(String("Wakeup likely cause: first boot after firmware flash, reason: ") + wakeup_reason,1);
      }
      break;
    }
    // needed?
    Serial.begin(115200);
    debugMessage(String("I'm back"),1);

    // after wakeup
    timeLastSleepMS = millis();
    powerLightWakeUp();
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

  debugMessage("screenAlert() start",2);

  display.setTextColor(ST77XX_WHITE);
  display.fillScreen(ST77XX_BLACK);

  debugMessage(String("screenAlert(): text to display is '") + messageText + "'",2);

  // does message fit on one line?
  display.setFont(&FreeSans18pt7b);
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

      display.setFont(&FreeSans12pt7b);
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
        display.setFont(&FreeSans18pt7b);
        display.getTextBounds(messageText.c_str(), 0, 0, &x1, &y1, &largeFontPhraseOneWidth, &largeFontPhraseOneHeight);
        display.setCursor(display.width()/2-largeFontPhraseOneWidth/2,display.height()/2+largeFontPhraseOneHeight/2);
        display.print(messageText);
      }
    }
  }
  debugMessage("screenAlert() end",2);
  return success;
}

void screenCurrentData()
// Display environmental information
{
  debugMessage("screenCurrentData() start",1);

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
  display.setTextColor(warningColor[co2Range(sensorData.ambientCO2[co2GraphPoints-1])]);  // Use highlight color look-up table
  display.print(sensorData.ambientCO2[co2GraphPoints-1],0);
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

  debugMessage("screenCurrentData() end",1);
}

void screenColor()
// Represents CO2 value on screen as a single color fill
{
  debugMessage("screenColor() start",1);
  display.fillScreen(warningColor[co2Range(sensorData.ambientCO2[co2GraphPoints-1])]);  // Use highlight color LUT
  debugMessage("screenColor() end",1);
}

void screenSaver()
// Display current CO2 reading at a random location (e.g. "screen saver")
{
  int16_t x, y;

  debugMessage("screenSaver() start",1);
  display.fillScreen(ST77XX_BLACK);
  display.setTextSize(1);  // Needed so custom fonts scale properly
  display.setFont(&FreeSans18pt7b);

  // Pick a random location that'll show up
  x = random(xMargins,display.width()-xMargins-64);  // 64 pixels leaves room for 4 digit CO2 value
  y = random(35,display.height()-yMargins); // 35 pixels leaves vertical room for text display
  display.setCursor(x,y);
  display.setTextColor(warningColor[co2Range(sensorData.ambientCO2[co2GraphPoints-1])]);  // Use highlight color LUT
  display.println(sensorData.ambientCO2[co2GraphPoints-1]);
  debugMessage("screenSaver() end", 1);
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

  debugMessage("screenAggregateData() start",2);
  display.fillScreen(ST77XX_BLACK);   // clear screen

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
  debugMessage("screenAggregateData() end",2);
}

void screenGraph()
// Description : Draw a graph of recent (CO2) values from right (most recent) to left. -1 values not graphed.
// Parameters: none
// Output : none
// Improvement : NA 
{
  uint8_t loop; // upper bound is co2GraphPoints definition
  int16_t x1, y1; // used by getTextBounds()
  uint16_t text1Width, text1Height, text2Width, text2Height; // used by getTextBounds()
  uint16_t deltaX, x, y, xp, yp;  // graphing positions
  uint16_t graphX0, graphY0, graphX1, graphY1;  // graphing area bounding box
  String minlabel, maxlabel, xlabel;
  float minvalue, maxvalue;
  bool firstpoint = true, nodata = true;

  debugMessage("screenGraph start",1);

  display.fillScreen(ST77XX_BLACK);
  display.setFont();
  display.setTextColor(ST77XX_WHITE);

  // Scan the retained CO2 data for max & min to scale the plot
  minvalue = sensorCO2Max;
  maxvalue = sensorCO2Min;
  for(loop=0;loop<co2GraphPoints;loop++) {
    if(sensorData.ambientCO2[loop] == -1) continue;   // Skip "empty" slots
    nodata = false;  // At least one data point
    if(sensorData.ambientCO2[loop] < minvalue) minvalue = sensorData.ambientCO2[loop];
    if(sensorData.ambientCO2[loop] > maxvalue) maxvalue = sensorData.ambientCO2[loop];
  }

  // do we have data? (e.g., just booted)
  if(nodata) {
    xlabel = String("Awaiting CO2 Values");  // Center overall graph label below the drawing area
  }
  else {
    xlabel = String("Recent CO2 values");  // Center overall graph label below the drawing area

    // since we have data, pad min and max CO2 to add room and be multiples of 50 (for nicer axis labels)
    minvalue = (uint16_t(minvalue)/50)*50;
    maxvalue = ((uint16_t(maxvalue)/50)+1)*50;
  }
  debugMessage(String("Min / max: ") + minvalue + " / " + maxvalue,2);

  display.getTextBounds(xlabel.c_str(),0,0,&x1,&y1,&text1Width,&text1Height);
  display.setCursor(((display.width()-text1Width)/2),(display.height()-(text1Height+yMargins)));
  display.print(xlabel);

  // Set drawing area bounding box value
  graphY1 = display.height() - text1Height - yMargins - 5;  // Room at the bottom for the graph label

  // calculate width and height of CO2 value labels
  minlabel = String(uint16_t(minvalue));
  maxlabel = String(uint16_t(maxvalue));
  display.getTextBounds(maxlabel.c_str(),0,0,&x1,&y1,&text1Width,&text1Height);
  display.getTextBounds(minlabel.c_str(),0,0,&x1,&y1,&text2Width,&text2Height);

  // Set drawing area bounding box values
  // calculate bounding box knowing max width and height of CO2 value labels
  graphX0 = (text1Width >= text2Width) ? xMargins + text1Width : xMargins + text2Width;
  graphY0 = yMargins;
  graphX1 = display.width() - xMargins;

  // Draw axis lines
  display.drawLine(graphX0,graphY0,graphX0,graphY1,ST77XX_BLUE);
  display.drawLine(graphX0,graphY1,graphX1,graphY1,ST77XX_BLUE);
  
  // Draw Y axis labels
  display.setTextColor(ST77XX_BLUE);
  display.setCursor(graphX0-xMargins-text1Width,yMargins);
  display.print(maxlabel);
  display.setCursor(graphX0-xMargins-text2Width,graphY1-text2Height); 
  display.print(minlabel);

  // Plot however many data points we have both with filled circles at each
  // point and lines connecting the points.  Color the filled circles with the
  // appropriate CO2 warning level color.
  deltaX = (graphX1 - graphX0 - 10) / (co2GraphPoints-1);  // X distance between points, 10 pixel padding for Y axis
  xp = graphX0;
  yp = graphY1;
  for(loop=0;loop<co2GraphPoints;loop++) {
    if(sensorData.ambientCO2[loop] == -1) continue;
    x = graphX0 + 10 + (loop*deltaX);  // Include 10 pixel padding for Y axis
    y = graphY1 - (((sensorData.ambientCO2[loop] - minvalue)/(maxvalue-minvalue)) * (graphY1-graphY0));
    debugMessage(String("Array ") + loop + " y value is " + y,2);
    display.fillCircle(x,y,4,warningColor[co2Range(sensorData.ambientCO2[loop])]);
    if(firstpoint) {
      // If this is the first drawn point then don't try to draw a line
      firstpoint = false;
    }
    else {
      // Draw line from previous point (if one) to this point
      display.drawLine(xp,yp,x,y,ST77XX_WHITE);
    }
    // Save x & y of this point to use as previous point for next one.
    xp = x;
    yp = y;
  }
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
    debugMessage(String("screenHelperBatteryStatus(): ") + hardwareData.batteryPercent + "%, " + int(0.5+(hardwareData.batteryPercent*((barWidth-4)/100.0))) + " of " + (barWidth-4) + " pixels",1);
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
    retainCO2(simulatedCO2);

    debugMessage(String("Simulated temp: ") + sensorData.ambientTemperatureF + "F, humidity: " + sensorData.ambientHumidity
      + "%, CO2: " + sensorData.ambientCO2[co2GraphPoints-1] + "ppm",1);
  }
#endif

bool batteryInit()
// Description: Initialize whatever battery monitoring sensor/controller is available
// Parameters: none
// Output : true if battery is ready to be monitored, false if not
// Improvement : NA
{
  debugMessage("batteryInit() start",2);

  bool success = false;
  #ifdef HARDWARE_SIMULATE
    success = true;  // Simulating, nothing to be done
  #else
    // Feather ESP32 V2 battery monitoring
    #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
      // Feather ESP32 V2 has no battery sensor/controller, just a simple
      // built-in voltage divider hooked to an analog pin
      success = true; // Nothing to do on ESP32 V2
    #endif
    // Feather ESP32S3 Rev TFT battery monitoring (via onboard MAX17048)
    #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT
      bool status;
      // initialize battery monitor
      status = lipoBattery.begin();
      if(!status) {
        // Couldn't initialize MAX17048, initialization failed
        debugMessage("batteryInit(): MAX17048 initialization failed",1);
      }
      else {
        // Q: setAlertVoltages, what values expected? 
        lipoBattery.setAlertVoltages((batteryVoltageMin/100.0),(batteryVoltageMax/100.0));
        debugMessage(String("batteryInit(): MAX1704X initialized, low voltage alert set to ") + (batteryVoltageMin/100.0) + "v",1);
        success = true;     
      }
    #endif
  #endif
  debugMessage("batteryInit() end",2);
  return success;
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
        debugMessage(String("batteryRead() from pin: ") + hardwareData.batteryVoltage + "v, " + hardwareData.batteryPercent + "%",1);
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
      debugMessage(String("batteryRead(): MAX17048: ") + hardwareData.batteryVoltage + "v, " + hardwareData.batteryPercent + "%",1);
      return true;
    #endif
  #endif
}

bool sensorSCD4xSSInit()
// Description: initialize SCD40
// Parameters: none
// Output : true if successful read, false if not
// Improvement : NA  
{
  debugMessage("sensorSCD4xSSInit() start",2);
  #ifdef HARDWARE_SIMULATE
    return true;
  #else
    char errorMessage[256];
    int16_t error;

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
    if (error) {
      errorToString(error, errorMessage, 256);
      debugMessage(String(errorMessage) + " executing SCD40 setSensorAltitude()",1);
      return false;
    }
    else {
      debugMessage(String("sensorSCD4xSSInit(): SCD40 altitude set to ") + SITE_ALTITUDE + " meters",2);
      return true;
    }

    // // Start Measurement.  For high power mode, with a fixed update interval of 5 seconds
    // // (the typical usage mode), use startPeriodicMeasurement().  For low power mode, with
    // // a longer fixed sample interval of 30 seconds, use startLowPowerPeriodicMeasurement()
    // // uint16_t error = co2Sensor.startPeriodicMeasurement();
    // error = co2Sensor.startLowPowerPeriodicMeasurement();
    // if (error) {
    //   errorToString(error, errorMessage, 256);
    //   debugMessage(String(errorMessage) + " executing SCD40 startLowPowerPeriodicMeasurement()",1);
    //   return false;
    // }
    // else
    // {
    //   debugMessage("SCD40 starting low power periodic measurements",1);
    //   return true;
    // }
  #endif
  debugMessage("sensorSCD4xSSInit() end",2);
}

bool sensorSCD4xSSRead()
// Description: single shot SCD40 read
// Parameters: none
// Output : true if successful read, false if not
// Improvement : NA  
{
  bool success = false;

  #ifdef HARDWARE_SIMULATE
    success = true;
    sensorSCD4xSimulate(1,5);
  #else
    uint16_t co2 = 0;
    float temperatureC = 0.0f;
    float humidity = 0.0f;

    int16_t error;
    char errorMessage[256];

    debugMessage("sensorSCD4xSSRead() start",2);
    
    // Wake the sensor up from sleep mode.
    error = co2Sensor.wakeUp();
    if (error) {
      errorToString(error, errorMessage, sizeof errorMessage);
      debugMessage(String("Error trying to execute wakeUp(): ") + errorMessage,1);
    }
    else {
      // Ignore first measurement after wake up.
      error = co2Sensor.measureSingleShot();
      if (error) {
        errorToString(error, errorMessage, sizeof errorMessage);
        debugMessage(String("Error trying to execute measureSingleShot(): ") + errorMessage,1);
      }
      else {
        // Perform single shot measurement and read data.
        error = co2Sensor.measureAndReadSingleShot(co2, temperatureC, humidity);
        if (error) {
          errorToString(error, errorMessage, sizeof errorMessage);
          debugMessage(String("Error trying to execute measureAndReadSingleShot(): ") + errorMessage,1);
        }
        else {
          if (co2 < sensorCO2Min || co2 > sensorCO2Max) {
            debugMessage(String("SCD40 CO2 reading: ") + co2 + " is out of expected range",1);
            //(sensorData.ambientCO2 < sensorCO2Min) ? sensorData.ambientCO2 = sensorCO2Min : sensorData.ambientCO2 = sensorCO2Max;
          }
          else {
            // Valid measurement available, update globals
            sensorData.ambientTemperatureF = (temperatureC*1.8)+32.0;
            sensorData.ambientHumidity = humidity;
            retainCO2(co2);
            debugMessage(String("sensorSCD4xSSRead(): ") + sensorData.ambientTemperatureF + "F, " + sensorData.ambientHumidity + "%, " + sensorData.ambientCO2[co2GraphPoints-1] + " ppm",1);
            success = true;
          }
        }
      }
    }
  #endif
  debugMessage("sensorSCD4xSSRead() end",2);
  return(success);
}

// Accumulate a CO2 value into the CO2 data array, LIFO queue
void retainCO2(uint16_t co2)
{
  for(uint8_t loop=1;loop<co2GraphPoints;loop++) {
    sensorData.ambientCO2[loop-1] = sensorData.ambientCO2[loop];
  }
  sensorData.ambientCO2[co2GraphPoints-1] = co2;
}

uint8_t co2Range(uint16_t co2) 
// converts co2 value to index value for labeling and color
{
  uint8_t co2Range = 
    (co2 <= sensorCO2Fair) ? 0 :
    (co2 <= sensorCO2Poor) ? 1 :
    (co2 <= sensorCO2Bad)  ? 2 : 3;

  debugMessage(String("co2Range(): CO2 value ") + co2 + " = co2Range " + co2Range, 2);
  return co2Range;
}

void powerLightSleep(uint32_t sleepTime)
// Description: power saving mode where display and ESP32 are disabled
// Parameters: light sleep time in microseconds
// Output : NA
// Improvement : NA  
{
  debugMessage(String("powerLightSleep() start"),1);

  // sleep the display
  display.enableSleep(true);
  delay(120);             // Wait for the display to enter sleep mode
  debugMessage(String("powerLightSleep(): display sleep"),1);

  // sleep the CO2 sensor
  #ifndef HARDWARE_SIMULATE
    static char errorMessage[64];
    static int16_t error;

    error = co2Sensor.powerDown();
    if (error) {
      errorToString(error, errorMessage, sizeof errorMessage);
      debugMessage(String("Error trying to execute powerDown(): ") + errorMessage,1);
    }
    else
      debugMessage("powerLightSleep(): SCD40 off",1);
  #endif

  //light sleep the ESP32
  debugMessage(String("powerLightSleep() end: ESP32 light sleep for ") + (sleepTime/1000000) + " seconds",1);
  esp_light_sleep_start();
}

void powerLightWakeUp()
// Description: 
// Parameters: none
// Output : true if successful read, false if not
// Improvement : NA  
{
  debugMessage(String("powerLightWakeUp() start"),1);

  // wake the display
  display.enableSleep(false);
  debugMessage(String("powerLightWakeUp(): display wakeup"),1);

  // wake up the CO2 sensor
  #ifndef HARDWARE_SIMULATE
    static char errorMessage[64];
    static int16_t error;

    error = co2Sensor.wakeUp();
    if (error) {
      errorToString(error, errorMessage, sizeof errorMessage);
      debugMessage(String("Error trying to execute wakeUp(): ") + errorMessage,1);
    }
    else
      debugMessage("powerLightWakeUp(): SCD40 on",1);
  #endif

  debugMessage(String("powerLightWakeUp() end"),1);
}

void powerDeepSleep(uint32_t sleepTime)
// Description: turn off component hardware, then put ESP32 into deep sleep mode for specified time
// Parameters: deep sleep time in microseconds
// Output : NA
// Improvement : MAX1704 powerdown, CO2 stopPeriodicMeasurement might not be needed
{
  debugMessage(String("powerDeepSleep start"),1);

  // stop potentially started SCD40 measurement
  #ifndef HARDWARE_SIMULATE
    co2Sensor.powerDown();
    debugMessage(String("powerDeepSleep(): SCD40 off"),2);
  #endif

  // power down MAX1704
  // Q: is this needed if I already powered down i2c?
  // lipoBattery.hibernate();
  // debugMessage("powerDeepSleep(): MAX1704X off",2);
  
  //Q: do these two screen related calls work on ESP32v2?

  // power down TFT screen
  digitalWrite(TFT_BACKLITE, LOW);
  digitalWrite(TFT_I2C_POWER, LOW);
  debugMessage(String("powerDeepSleep(): display off"),2);

  // hardware specific powerdown routines
  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    // Turn off the I2C power
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, LOW);
    debugMessage(String("powerDeepSleep(): ESP32V2 i2c off"),2);
  #endif

  esp_sleep_enable_timer_wakeup(sleepTime);
  debugMessage(String("powerDeepSleep() end: ESP32 deep sleep for ") + (sleepTime/1000000) + " seconds",1);
  esp_deep_sleep_start();
}

float randomFloatRange(uint16_t minVal, uint16_t maxVal) {
  // Scale the range up to hundredths
  uint32_t scaled = random((maxVal - minVal) * 100 + 1);  
  return minVal + (scaled / 100.0f);
}

void debugMessage(String messageText, uint8_t messageLevel)
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