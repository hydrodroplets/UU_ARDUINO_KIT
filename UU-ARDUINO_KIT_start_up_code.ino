////////////////////// UU-ARDUINO KIT start up code
/*!
 * @file mprls_simpletest.ino
 *
 * UU-ARDUINO KIT start up code 
 * 
 * This is the start up code of the UU-Arduino KIT. You can use and adjust the code 
 * to your own need to sense different variables. 
 * 
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * MPLRS and BMP Written by Limor Fried/Ladyada for Adafruit Industries. 
 * Pieced together and extend by BMCF21 
 *
 * MIT license, all text here must be included in any redistribution.
 *
 * Version dd. 20210426
 * 
 */
 
////////////////////// Include different libraries 
//
//  in case you miss lib's see Sketch > Include libraries > Manage libraries > copy paste missing lib and install

#include <Wire.h>
#include "Adafruit_MPRLS.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

#include <SD.h>
 
#include <RTClib.h>
#include <SPI.h>

#include <Sleep_n0m1.h> // sleep library 
  
 
//
////////////////////// Define parameters
//
//

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
 
#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

//RTC_DS1307 RTC; // define the Real Time Clock object
RTC_PCF8523 RTC; // for the data logging shield, we use digital pin 10 for the SD cs line

// the logging file
File logfile;

Sleep sleep;
unsigned long sleepTime = 2000; //300000; //300000; //60000; //set sleep time in ms, max sleep time is 49.7 days
// 1000 mils = 1 s
// 60000 = 60 s = 1 min
// 3600000 = 60 min 
  
////////////////////// Setup Arduino
//
//
void setup() {
  
  Serial.begin(115200);
  Wire.begin();                                      //begin the wire communication

  if(!bmp.begin())   // Initialise the BMPsensor
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  displaySensorDetails(); // Display some basic information on this sensor and comment if not needed
    
  Serial.println("MPRLS Simple Test");
  if (!mpr.begin()) {Serial.println("Failed to communicate with MPRLS sensor, check wiring?");// Initialise the BMPsensor
  }
 SDINI();       // Initialise the SD shield with RTC
}

void loop() {

///////////////////////// Sleep mode to save pwr
//
  sleep.pwrDownMode(); //set sleep mode
  sleep.sleepDelay(sleepTime); //sleep for: sleepTime
  Serial.print(F(":O good morning \n "));
    
///////////////////////// TIME and date write
//
// delay for the amount of time we want between readings delay((LOG_INTERVAL - 1) - (millis() % LOG_INTERVAL));
  DateTime now;
    // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");
  #if ECHO_TO_SERIAL
    Serial.print(m);         // milliseconds since start
    Serial.print(F(", "));
  #endif
  
  // fetch the time
  now = RTC.now();
  // log time
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
  #if ECHO_TO_SERIAL
      Serial.print(now.unixtime()); // seconds since 1/1/1970
      Serial.print(F(", "));
      Serial.print('"');
      Serial.print(now.year(), DEC);
      Serial.print(F("/"));
      Serial.print(now.month(), DEC);
      Serial.print(F("/"));
      Serial.print(now.day(), DEC);
      Serial.print(F(" "));
      Serial.print(now.hour(), DEC);
      Serial.print(F(":"));
      Serial.print(now.minute(), DEC);
      Serial.print(F(":"));
      Serial.print(now.second(), DEC);
      Serial.print('"');
   #endif //ECHO_TO_SERIAL


///////////////////////
// MPRLS
  float pressure_hPa = mpr.readPressure();
  Serial.print("Pressure (hPa): "); Serial.println(pressure_hPa);

 
// BMP180
  /* Get a new sensor event */ 
  sensors_event_t event;
  bmp.getEvent(&event);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    Serial.print("Pressure:    ");
    Serial.print(event.pressure);
    Serial.println(" hPa");
    Serial.print('\n');
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */
  }
  else
  {
    Serial.println("Sensor error");
  }   
    /* First we get the current temperature from the BMP085 */
  float temperature;
  bmp.getTemperature(&temperature);
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

    /* Then convert the atmospheric pressure, and SLP to altitude         */
    /* Update this next line with the current SLP for better results      */
  float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
  Serial.print("Altitude:    "); 
  Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure)); 
  Serial.println(" m");
  Serial.println("");
   
  Serial.print('\n');
  Serial.print("WL (mm): "); Serial.println((pressure_hPa-(event.pressure-(abs(pressure_hPa-event.pressure))))* 0.0102*1000);
  Serial.print('\n');
  Serial.print('\n');
  delay(1000);

  ///////////////////////// WRITE SD
  // in case you have more / less variables you need to adjust here
  //
    logfile.print(", ");
    logfile.print((pressure_hPa-(event.pressure-(abs(pressure_hPa-event.pressure))))* 0.0102*1000);
    logfile.print(", ");
    logfile.print(event.pressure);
    logfile.print(", ");
    logfile.print(temperature);
    logfile.println();    
    logfile.flush();   
}


void displaySensorDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);


  sensors_event_t event;
  bmp.getEvent(&event);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    Serial.print("Pressure:    ");
    Serial.print(event.pressure);
    Serial.println(" hPa");
    
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */
     
    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");

    /* Then convert the atmospheric pressure, and SLP to altitude         */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print("Altitude:    "); 
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure)); 
    Serial.println(" m");
    Serial.println("");
  }
  else
  {
    Serial.println("Sensor error");
  }
  delay(1000);
  
}

/// Initialize SD card
void SDINI() {
  Serial.begin(115200);
  Serial.println();

  Serial.print(F("Initializing SD card..."));
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  Serial.begin(115200);
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(10,11,12,9)) {}    // double check that the wires are correctly connected from SD shield to arduino pins 10,11,12,9!!! 
  Serial.println(F("card initialized."));
  // create a new file
  char filename[] = "LOGGER00.csv";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }
    Serial.print(F("Logging to: "));
    Serial.println(filename);

  // connect to RTC
  Wire.begin();
  if (!RTC.begin()) {
    logfile.println(F("RTC failed"));
#if ECHO_TO_SERIAL
    Serial.println(F("RTC failed"));
#endif  //ECHO_TO_SERIAL
  }

  logfile.println("millis,stamp,datetime,P_WL[mm],P_air[hPa], T [C]");  // if you have more variables you need to add / adjust here
#if ECHO_TO_SERIAL
  Serial.println(F("millis,stamp,datetime,P_WL[mm],P_air [hPa],deg C")); // if you have more variables you need to add / adjust here
#endif //ECHO_TO_SERIAL
}
