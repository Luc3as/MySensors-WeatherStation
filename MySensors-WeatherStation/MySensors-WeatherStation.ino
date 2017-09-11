#include <Arduino.h>

/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 ******************************
 *
 */
// Enable MY_DEBUG flag for MY_DEBUG prints. This will add a lot to the size of the final sketch but good
// to see what is actually is happening when developing
#define MY_DEBUG


// Enable and select radio type attached
#define MY_RADIO_NRF24

#define MY_NODE_ID 5
#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_BARO 2
#define CHILD_ID_FOREC 3
#define CHILD_ID_VBATT 4
#define CHILD_ID_VSOLAR 5
#define CHILD_ID_MOISTURE 6
#define CHILD_ID_RAIN_LOG 7  // Keeps track of accumulated rainfall
#define CHILD_ID_RAIN_RATE 8 // rain rate
//#define CHILD_ID_TRIPPED_INDICATOR 8  // Indicates Tripped when rain detected


#include <SPI.h>
#include <MySensors.h>
#include <Wire.h>
#include <SI7021.h>
#include <Adafruit_BMP085.h>
#include <math.h>
#include <TimeLib.h>


#define CALIBRATE_FACTOR 15 // amount of rain per rain bucket tip e.g. 5 is .05mm
#define EEPROM_BUFFER_LOCATION 0  // location of the EEPROM circular buffer
#define E_BUFFER_LENGTH 240
#define RAIN_BUCKET_SIZE 120
int BATTERY_SENSE_PIN = A2;  // select the input pin for the battery sense point
int SOLAR_SENSE_PIN = A3;  // select the input pin for the solar panel sense point
int MOISTURE_SENSE_PIN = A1;  // select the input pin for the soil mosture sensor
int TIP_SENSOR_PIN = 3; // input pin where reed switch or hall effect sensor is attached
int eepromIndex;
#define WAIT_BETWEEN_SEND 200  // time between sending messages, allowing to clear buffers on radio and GW

// Constants
#define BATTERY_MAX_VOLTAGE 4.21 // V, nominal voltage at battery for powering up device
#define BATTERY_MIN_VOLTAGE 2.9 // V, minimal voltage at which sensor still works
unsigned long SLEEP_TIME = 60000UL; // Sleep time between reads (in milliseconds)
int SEND_DATA_EVERY = 1 ; // send data to controller every n minutes
const int ALTITUDE = 242; // <-- adapt this value to your own location's altitude.

const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
  STABLE = 0,     // "Stable Weather Pattern"
  SUNNY = 1,      // "Slowly rising Good Weather", "Clear/Sunny "
  CLOUDY = 2,     // "Slowly falling L-Pressure ", "Cloudy/Rain "
  UNSTABLE = 3,   // "Quickly rising H-Press",     "Not Stable"
  THUNDERSTORM = 4, // "Quickly falling L-Press",    "Thunderstorm"
  UNKNOWN = 5     // "Unknown (More Time needed)
};

const unsigned long oneHour = 3600;
unsigned long lastTipTime;
unsigned long lastTipTimeRTC;
unsigned long thisTipTime;
unsigned long lastRainTime = 0; //Used for rainRate calculation
unsigned int rainBucket [RAIN_BUCKET_SIZE] ; /* 24 hours x 5 Days = 120 hours */
unsigned int rainRate = 0;
uint8_t rainWindow = 72;         //default rain window in hours.  Will be overwritten with msgTrippedVar1.
int wasTippedBuffer = 0;
int rainSensorThreshold = 50; //default rain sensor sensitivity in hundredths.  Will be overwritten with msgTrippedVar2.
uint8_t state = 0;
uint8_t oldState = 2; //Setting the default to something other than 1 or 0
unsigned int lastRainRate = 0;
int lastMeasure = 0;
bool gotTime = false;
uint8_t lastHour = 0;
uint8_t currentHour;
int lulaby = 0;
int index = 0;
float lastPressureAvg = 0;
float change = 0;
float pressure = 0;

Adafruit_BMP085 bmp;
float lastTemperature;
float lastHumidity;
boolean metric = true;
int oldBatteryPcnt = 0;
float oldVBatt = 0;
float oldVSolar = 0;
int oldMoisture = 0;
float lastPressure = -1;
int lastForecast = -1;
int forecast = -1;

const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];

// this CONVERSION_FACTOR is used to convert from Pa to kPa in forecast algorithm
// get kPa/h be dividing hPa by 10
#define CONVERSION_FACTOR (1.0/10.0)

SI7021 sisensor;

int minuteCount = 0;
int sendCount = -1;
bool firstRound = true;
// average value is used in forecast algorithm.
float pressureAvg;
// average after 2 hours is used as reference value for the next iteration.
float pressureAvg2;
float dP_dt;

MyMessage msgHumidity(CHILD_ID_HUM, V_HUM);
MyMessage msgTemperature(CHILD_ID_TEMP, V_TEMP);
MyMessage pressureMsg(CHILD_ID_BARO, V_PRESSURE);
MyMessage forecastMsg(CHILD_ID_FOREC, V_FORECAST);
MyMessage msgPrefix(CHILD_ID_BARO, V_UNIT_PREFIX);  // Custom unit message.
MyMessage msgVBatt(CHILD_ID_VBATT, V_VOLTAGE);
MyMessage msgVSolar(CHILD_ID_VSOLAR, V_VOLTAGE);
MyMessage msgMoisture(CHILD_ID_MOISTURE, V_LEVEL);
MyMessage msgMoisturePrefix(CHILD_ID_MOISTURE, V_UNIT_PREFIX);  // Custom unit message.
MyMessage msgRainRate(CHILD_ID_RAIN_RATE, V_RAINRATE);
MyMessage msgRain(CHILD_ID_RAIN_LOG, V_RAIN);
MyMessage msgPrefix1(CHILD_ID_RAIN_LOG, V_UNIT_PREFIX);  // Custom unit message.
MyMessage msgRainVAR1(CHILD_ID_RAIN_LOG, V_VAR1);
MyMessage msgRainVAR2(CHILD_ID_RAIN_LOG, V_VAR2);
MyMessage msgRainVAR3(CHILD_ID_RAIN_LOG, V_VAR3);
MyMessage msgRainVAR4(CHILD_ID_RAIN_LOG, V_VAR4);
MyMessage msgRainVAR5(CHILD_ID_RAIN_LOG, V_VAR5);
//MyMessage msgTripped(CHILD_ID_TRIPPED_INDICATOR, V_TRIPPED);
//MyMessage msgTrippedVar1(CHILD_ID_TRIPPED_INDICATOR, V_VAR1);
//MyMessage msgTrippedVar2(CHILD_ID_TRIPPED_INDICATOR, V_VAR2);


void presentation()
{
  // Send the Sketch Version Information to the Gateway
  sendSketchInfo("Weather_Station", "2.1");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM, S_HUM);
  wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
  present(CHILD_ID_TEMP, S_TEMP);
  wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
  present(CHILD_ID_BARO, S_BARO);
  wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
  present(CHILD_ID_FOREC, S_BARO);
  wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
  present(CHILD_ID_VBATT, S_MULTIMETER);
  wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
  present(CHILD_ID_VSOLAR, S_MULTIMETER);
  wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
  present(CHILD_ID_MOISTURE, S_MOISTURE);
  wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
  present(CHILD_ID_RAIN_LOG, S_RAIN);
  wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
  present(CHILD_ID_RAIN_RATE, S_RAIN);

  //present(CHILD_ID_TRIPPED_INDICATOR, S_MOTION);
  #ifdef MY_DEBUG
    Serial.println(F("Presentation complete"));
  #endif
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
// SETUP
//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void setup()
{
  // use the 1.1 V internal reference for voltage measurement
  #if defined(__AVR_ATmega2560__)
     analogReference(INTERNAL1V1);
  #else
     analogReference(INTERNAL);
  #endif

  // setup BMP sensor
  if (!bmp.begin())
  {
    Serial.println(F("Could not find a valid BMP180 sensor, check wiring!"));
  }
  metric = getControllerConfig().isMetric;

  // setup SI7027 sensor
  if (!sisensor.begin())
  {
    Serial.println(F("Could not find a valid SI7021 sensor, check wiring!"));
  }

  // Set up the IO
  pinMode(TIP_SENSOR_PIN, INPUT_PULLUP);
  //attachInterrupt (digitalPinToInterrupt(TIP_SENSOR_PIN), sensorTipped, FALLING);  // depending on location of the hall effect sensor may need CHANGE

  //retrieve from EEPROM stored values on a power cycle.
  bool isDataOnEeprom = false;
  for (int i = 0; i < E_BUFFER_LENGTH; i++)
  {
    uint8_t locator = loadState(EEPROM_BUFFER_LOCATION + i);
    if (locator == 0xFE)  // found the EEPROM circular buffer index
    {
      eepromIndex = EEPROM_BUFFER_LOCATION + i;
      #ifdef MY_DEBUG
        Serial.print(F("EEPROM Index "));
        Serial.println(eepromIndex);
      #endif
      //Now that we have the buffer index let's populate the rainBucket[] with data from eeprom
      loadRainArray(eepromIndex);
      isDataOnEeprom = true;
      break;
    }
  }
  // Added for the first time it is run on a new Arduino
  if (!isDataOnEeprom)
  {
    #ifdef MY_DEBUG
      Serial.print(F("I didn't find valid EEPROM Index, so I'm writing one to location 0"));
    #endif
    eepromIndex = EEPROM_BUFFER_LOCATION;
    saveState(eepromIndex, 0xFE);
    saveState(eepromIndex + 1, 0xFE);
    //then I will clear out any bad data
    for (int i = 2; i <= E_BUFFER_LENGTH; i++)
    {
      saveState(i, 0x00);
    }
  }
  lastTipTime = millis();
  lastTipTimeRTC = now();
  //
  //request(CHILD_ID_TRIPPED_INDICATOR, V_VAR1);
  //request(CHILD_ID_TRIPPED_INDICATOR, V_VAR2);
  //

  transmitRainData(); //Setup complete send any data loaded from eeprom to gateway

} // END Setup


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// LOOP
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


void loop() {
  #ifdef MY_DEBUG
    Serial.print(F("Wakeup reason : "));
    if (lulaby == 0) {
      Serial.println(F("first wakeup."));
    } else if (lulaby == 1 ) {
      Serial.println(F("interrupt 1."));
    } else {
      Serial.println(F("sleep timer."));
    }
  #endif

  if (lulaby == 1) // running when interrupt wakes up by raingauge tip
  {
    thisTipTime = millis();
    //if (thisTipTime - lastTipTime > 50UL) { // debounce raingauge
      rainBucket[0] += CALIBRATE_FACTOR; // adds CALIBRATE_FACTOR hundredths of unit each tip
      wasTippedBuffer++;
      #ifdef MY_DEBUG
        Serial.println(F("It is raining..."));
      #endif
    //}
    lastTipTime = thisTipTime;
    lastTipTimeRTC = now();
    if (lastRainTime == 0) {
      lastRainTime = now() ;
    }
    setTime(now()+2); // add time when it is awaken now

  } else {  // running when node woke up from timer

    if (gotTime == true) {  // time is set, move time forward with SLEEP_TIME
      #ifdef MY_DEBUG
        Serial.println(F("Adding SLEEP_TIME to actual time"));
      #endif
      setTime(now()+(SLEEP_TIME / 1000));
    } else {  // time is not set, request new time
      #ifdef MY_DEBUG
        Serial.println(F("Requesting time"));
      #endif
      requestTime();
    }

  #ifdef MY_DEBUG
    // print current time we have from controller
    char currentTime[10];
    sprintf(currentTime, "%d:%d:%d",hour(),minute(),second());
    Serial.print(F("Current time on arduino  "));
    Serial.println(currentTime);
  #endif

  // increment sendCount every minute
  sendCount ++ ;
  if (sendCount >= SEND_DATA_EVERY || sendCount <= 0) { // send data every n minute
    sendCount = 0 ;

    #ifdef MY_DEBUG
      Serial.println(F("Measurring everything"));
    #endif

    // get the battery Voltage
    int batteryRead = analogRead(BATTERY_SENSE_PIN);

    // voltage divider across battery and using internal ADC ref voltage of 1.1V
    // Sense point is bypassed with 0.1 uF cap to reduce noise at that point
    // 3.94 M ohm a 421 k ohm , ref 1,1v Vmax at battery 11.395V , use http://www.raltron.com/cust/tools/voltage_divider.asp for max V calculation
    // 11.395/1023 = Volts per bit = 0,01113880742913000977517106549365
    float batteryV  = batteryRead * 0.01113880742913000977517106549365;
    int batteryPcnt = ((batteryV - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100 ;
    if (batteryPcnt <= 0 ) {
      batteryPcnt = 0;
    }

    #ifdef MY_DEBUG
      Serial.print(F("Analog read battery: "));
      Serial.println(batteryRead);

      Serial.print(F("Battery Voltage: "));
      Serial.print(batteryV);
      Serial.println(" V");

      Serial.print(F("Battery percent: "));
      Serial.print(batteryPcnt);
      Serial.println(F(" %"));
    #endif

    // send battery %
    if (oldBatteryPcnt != batteryPcnt) {
      sendBatteryLevel(batteryPcnt);
      wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
      oldBatteryPcnt = batteryPcnt;
    }

    // send battery voltage
    if (oldVBatt != batteryV) {
      send(msgVBatt.set(batteryV, 2));
      wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
      oldVBatt = batteryV;
    }

    // get the Solar pannel Voltage
    int solarReading = analogRead(SOLAR_SENSE_PIN);

    // voltage divider across battery and using internal ADC ref voltage of 1.1V
    // Sense point is bypassed with 0.1 uF cap to reduce noise at that point
    // 3.94 M ohm a 421 k ohm , ref 1,1v Vmax at battery 11.395V , use http://www.raltron.com/cust/tools/voltage_divider.asp for max v calculation
    // 11.395/1023 = Volts per bit = 0,01113880742913000977517106549365
    float solarV  = solarReading * 0.01113880742913000977517106549365;

    #ifdef MY_DEBUG
      Serial.print(F("Analog read solar pannel: "));
      Serial.println(solarReading);

      Serial.print(F("Solar pannel Voltage: "));
      Serial.print(solarV);
      Serial.println(F(" V"));
    #endif

    // send solar pannel voltage
    if (oldVSolar != solarV) {
      send(msgVSolar.set(solarV, 2));
      wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
      oldVSolar = solarV;
    }

    // BMP180 sensor
    // get the pressure reading
    pressure = 0 ;
    bmp.readTemperature();
    wait(1000, C_INTERNAL, I_TIME);
    pressure = bmp.readSealevelPressure(ALTITUDE) / 100.0;

    forecast = sample(pressure);

    #ifdef MY_DEBUG
      Serial.print(F("Pressure = "));
      Serial.print(pressure);
      Serial.println(F(" hPa"));
      Serial.print(F("Forecast = "));
      Serial.println(weather[forecast]);

      Serial.print(F("Temperature BMP = "));
      Serial.print(bmp.readTemperature());
      Serial.println(F(" *C"));

      Serial.print(F("Pressure = "));
      Serial.print(bmp.readPressure());
      Serial.println(F(" Pa"));

      // Calculate altitude assuming 'standard' barometric
      // pressure of 1013.25 millibar = 101325 Pascal
      Serial.print(F("Altitude = "));
      Serial.print(bmp.readAltitude());
      Serial.println(F(" meters"));

    // you can get a more precise measurement of altitude
    // if you know the current sea level pressure which will
    // vary with weather and such. If it is 1015 millibars
    // that is equal to 101500 Pascals.
      Serial.print(F("Real altitude = "));
      Serial.print(bmp.readAltitude(101700));
      Serial.println(F(" meters"));
    #endif

    // send pressure reading
    if (pressure != lastPressure)
    {
      send(pressureMsg.set(pressure, 1));
      wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
      send(msgPrefix.set("hPa"));  // Set custom unit.
      wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
      lastPressure = pressure;
    }

    // send forecast reading
    if (forecast != lastForecast)
    {
      send(forecastMsg.set(weather[forecast]));
      wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
      lastForecast = forecast;
    }

    // get the moisture reading
    int moistureRead = analogRead(MOISTURE_SENSE_PIN);

    int moisturePcnt = map(moistureRead, 0, 1024, 0, 101);

    #ifdef MY_DEBUG
      Serial.print(F("Analog read moisture: "));
      Serial.println(moistureRead);

      Serial.print(F("Moisture percent: "));
      Serial.print(moisturePcnt);
      Serial.println(F(" %"));
    #endif

    // send moisture %
    if (moisturePcnt != oldMoisture) {
      send(msgMoisture.set(moisturePcnt, 0));
      wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
      send(msgMoisturePrefix.set("%"));  // Set custom unit.
      wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
      oldMoisture = moisturePcnt;
    }

    // SI7027 Sensor
    // temperature
    float temperature = sisensor.getCelsiusHundredths();
    temperature = temperature /100 ;

    // humidity
    int  humidity = sisensor.getHumidityPercent();


    #ifdef MY_DEBUG
      Serial.print(F("Temperature SI7020 = "));
      Serial.print(temperature);
      Serial.println(F(" *C"));

      Serial.print(F("Humidity SI7020 = "));
      Serial.print(humidity);
      Serial.println(F(" %"));
    #endif

    // send temperature reading
    if (temperature != lastTemperature)
    {
      send(msgTemperature.set(temperature, 1));
      wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
      lastTemperature = temperature;
    }

    // send humidity reading
    if (humidity != lastHumidity)
    {
      send(msgHumidity.set(humidity, 0));
      wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
      lastHumidity = humidity;
    }

    // Rainbucket reading
    if (gotTime == true) {

      // let's constantly check to see if the rain in the past rainWindow hours is greater than rainSensorThreshold
      int measure = 0; // Check to see if we need to show sensor tripped in this block
      for (int i = 0; i < rainWindow; i++)
      {
        measure += rainBucket [i];
        if (measure != lastMeasure)
        {
          //      DEBUG_PRINT(F("measure value (total rainBucket within rainWindow): "));
          //      DEBUG_PRINTLN(measure);
          lastMeasure = measure;
        }
      }
      state = (measure >= (rainSensorThreshold * 100));

      // send rain tripped
      if (state != oldState)
      {
        //send(msgTripped.set(state));
        #ifdef MY_DEBUG
          Serial.print(F("New Sensor State... Sensor: "));
          Serial.println(state ? "Tripped" : "Not Tripped");
        #endif
        oldState = state;
      }

      unsigned long tipDelay = now() - lastRainTime;
      if (wasTippedBuffer) // if was tipped, then update the 24hour total and transmit to controller
      {
        #ifdef MY_DEBUG
          Serial.println(F("Sensor Tipped"));
          Serial.print(F("rainBucket [0] value: "));
          Serial.println(rainBucket [0]);
        #endif
        send(msgRain.set((float)rainTotal(currentHour) / 100, 1)); //Calculate the total rain for the day
        wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
        send(msgPrefix1.set("mm/h"));  // Set custom unit.
        wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
        wasTippedBuffer--;
        rainRate = ((oneHour) / tipDelay);

        // send rain rate
        if (rainRate != lastRainRate)
        {
          send(msgRainRate.set(rainRate, 1));
          wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
          #ifdef MY_DEBUG
            Serial.print(F("RainRate= "));
            Serial.println(rainRate);
          #endif
          lastRainRate = rainRate;
        }
        lastRainTime = lastTipTimeRTC;
      }

      currentHour = hour();
      if (currentHour != lastHour)  //  we have new hour , do the math
      {
        send(msgRain.set((float)rainTotal(currentHour) / 100, 1)); // send today's rainfall
        wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
        saveState(eepromIndex, highByte(rainBucket[0]));
        saveState(eepromIndex + 1, lowByte(rainBucket[0]));
        #ifdef MY_DEBUG
          Serial.println(F("One hour elapsed."));
          Serial.print(F("Saving rainBucket[0] to eeprom. rainBucket[0] = "));
          Serial.println(rainBucket[0]);
        #endif
        for (int i = RAIN_BUCKET_SIZE - 1; i >= 0; i--)//cascade an hour of values back into the array
        {
          rainBucket [i + 1] = rainBucket [i];
        }
        //request(CHILD_ID_TRIPPED_INDICATOR, V_VAR1);
        //request(CHILD_ID_TRIPPED_INDICATOR, V_VAR2);
        rainBucket[0] = 0;
        eepromIndex = eepromIndex + 2;
        if (eepromIndex > EEPROM_BUFFER_LOCATION + E_BUFFER_LENGTH)
        {
          eepromIndex = EEPROM_BUFFER_LOCATION;
        }
        #ifdef MY_DEBUG
          Serial.print(F("Writing to EEPROM.  Index: "));
          Serial.println(eepromIndex);
        #endif
        saveState(eepromIndex, 0xFE);
        saveState(eepromIndex + 1, 0xFE);
        gotTime = false;
        requestTime(); // sync the time every hour
        #ifdef MY_DEBUG
          Serial.println(F("Requesting time for this hour"));
        #endif
        transmitRainData();
        rainRate = 0;
        send(msgRainRate.set(rainRate, 1));
        wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
        #ifdef MY_DEBUG
          Serial.println(F("Sending rainRate is 0 to controller"));
        #endif
        lastHour = hour();
      }
    }
  } else {
    #ifdef MY_DEBUG
      Serial.println(F("Measurring pressure"));
    #endif
    // get the pressure reading but don't send data
    pressure = 0 ;
    bmp.readTemperature();
    wait(1000, C_INTERNAL, I_TIME);
    pressure = bmp.readSealevelPressure(ALTITUDE) / 100.0;
    forecast = sample(pressure);
  }

  // wait for time from controller
  // I have some issues with GW and NRF24 time was not served in time due to high amount of radio traffic
  unsigned long functionTimeout = millis();
  if (gotTime == false) {
    #ifdef MY_DEBUG
      Serial.print(F("Waiting for time to arrive from controller "));
    #endif
    while (gotTime == false && millis() - functionTimeout < 30000UL)
    {
      wait(1000, C_INTERNAL, I_TIME); // call once per second
      #ifdef MY_DEBUG
        Serial.print(F("."));
      #endif
    }
    #ifdef MY_DEBUG
      Serial.println(F(""));
    #endif
  }
}
  // all done, sleep a bit
  #ifdef MY_DEBUG
    Serial.println(F("=========================== sweet dreams  ============================="));
  #endif
  lulaby =  smartSleep(1, 0, SLEEP_TIME); //sleep a bit
} // End Loop


//
// Functions for pressure readings and weather forecast
//

float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
  {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }
  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  return lastPressureSamplesAverage;
}

// Algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure)
{
	// Calculate the average of the last n minutes.
	index = minuteCount % LAST_SAMPLES_COUNT;
	lastPressureSamples[index] = pressure;

	minuteCount++;
	if (minuteCount > 185)
	{
		minuteCount = 6;
	}

	if (minuteCount == 5)
	{
		pressureAvg = getLastPressureSamplesAverage();
	}
	else if (minuteCount == 35)
	{
		lastPressureAvg = getLastPressureSamplesAverage();
		change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change * 2; // note this is for t = 0.5hour
		}
		else
		{
			dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
		}
	}
	else if (minuteCount == 65)
	{
		lastPressureAvg = getLastPressureSamplesAverage();
		change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) //first time initial 3 hour
		{
			dP_dt = change; //note this is for t = 1 hour
		}
		else
		{
			dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 95)
	{
		lastPressureAvg = getLastPressureSamplesAverage();
		change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 1.5; // note this is for t = 1.5 hour
		}
		else
		{
			dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 125)
	{
		lastPressureAvg = getLastPressureSamplesAverage();
		pressureAvg2 = lastPressureAvg; // store for later use.
		change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 2; // note this is for t = 2 hour
		}
		else
		{
			dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 155)
	{
		lastPressureAvg = getLastPressureSamplesAverage();
		change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 2.5; // note this is for t = 2.5 hour
		}
		else
		{
			dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 185)
	{
		lastPressureAvg = getLastPressureSamplesAverage();
		change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 3; // note this is for t = 3 hour
		}
		else
		{
			dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
		}
		pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
		firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
	}

	int forecast = UNKNOWN;
	if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
	{
		forecast = UNKNOWN;
	}
	else if (dP_dt < (-0.25))
	{
		forecast = THUNDERSTORM;
	}
	else if (dP_dt > 0.25)
	{
		forecast = UNSTABLE;
	}
	else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
	{
		forecast = CLOUDY;
	}
	else if ((dP_dt > 0.05) && (dP_dt < 0.25))
	{
		forecast = SUNNY;
	}
	else if ((dP_dt >(-0.05)) && (dP_dt < 0.05))
	{
		forecast = STABLE;
	}
	else
	{
		forecast = UNKNOWN;
	}

	return forecast;

  #ifdef MY_DEBUG
    Serial.print(F("Forecast at minute "));
    Serial.print(minuteCount);
    Serial.print(F(" dP/dt = "));
    Serial.print(dP_dt);
    Serial.print(F("kPa/h --> "));
    Serial.println(weather[forecast]);
  #endif
  return forecast;
} // End sample


//
// Functions for Rain bucket readings and calculations
//

int rainTotal(int hours)
{
  int total = 0;
  for ( int i = 0; i <= hours; i++)
  {
    total += rainBucket [i];
  }
  return total;
} // END rainTotal

void loadRainArray(int eValue) // retrieve stored rain array from EEPROM on powerup
{
  for (int i = 1; i < RAIN_BUCKET_SIZE; i++)
  {
    eValue = eValue - 2;
    if (eValue < EEPROM_BUFFER_LOCATION)
    {
      eValue = EEPROM_BUFFER_LOCATION + E_BUFFER_LENGTH;
    }
    #ifdef MY_DEBUG
      //Serial.print(F("EEPROM location: "));
      //Serial.println(eValue);
    #endif
    uint8_t rainValueHigh = loadState(eValue);
    uint8_t rainValueLow = loadState(eValue + 1);
    unsigned int rainValue = rainValueHigh << 8;
    rainValue |= rainValueLow;
    rainBucket[i] = rainValue;
    #ifdef MY_DEBUG
      //Serial.print(F("rainBucket[ value: "));
      //Serial.print(i);
      //Serial.print(F("] value: "));
      //Serial.println(rainBucket[i]);
    #endif
  }
} // End loadRainArray

void transmitRainData(void)
{
  #ifdef MY_DEBUG
    Serial.print(F("In transmitRainData. currentHour = "));
    Serial.println(currentHour);
  #endif
  int rainUpdateTotal = 0;
  for (int i = currentHour; i >= 0; i--)
  {
    rainUpdateTotal += rainBucket[i];
    #ifdef MY_DEBUG
      //Serial.print(F("Adding rainBucket["));
      //Serial.print(i);
      //Serial.println(F("] to rainUpdateTotal."));
    #endif
  }
  #ifdef MY_DEBUG
    Serial.print(F("TX Day 1: rainUpdateTotal = "));
    Serial.println((float)rainUpdateTotal / 100.0);
  #endif
  send(msgRainVAR1.set((float)rainUpdateTotal / 100.0, 1)); //Send current day rain totals (resets at midnight)
  wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
#ifdef USE_DAILY
  rainUpdateTotal = 0;
#endif
  for (int i = currentHour + 24; i > currentHour; i--)
  {
    rainUpdateTotal += rainBucket[i];
    #ifdef MY_DEBUG
      //Serial.print(F("Adding rainBucket["));
      //Serial.print(i);
      //Serial.println(F("] to rainUpdateTotal."));
    #endif
  }
  #ifdef MY_DEBUG
    Serial.print(F("TX Day 2: rainUpdateTotal = "));
    Serial.println((float)rainUpdateTotal / 100.0);
  #endif
  send(msgRainVAR2.set((float)rainUpdateTotal / 100.0, 1));
  wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
#ifdef USE_DAILY
  rainUpdateTotal = 0;
#endif
  for (int i = currentHour + 48; i > currentHour + 24; i--)
  {
    rainUpdateTotal += rainBucket[i];
    #ifdef MY_DEBUG
      //Serial.print(F("Adding rainBucket["));
      //Serial.print(i);
      //Serial.println(F("] to rainUpdateTotal."));
    #endif
  }
  #ifdef MY_DEBUG
    Serial.print(F("TX Day 3: rainUpdateTotal = "));
    Serial.println((float)rainUpdateTotal / 100.0);
  #endif
  send(msgRainVAR3.set((float)rainUpdateTotal / 100.0, 1));
  wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
#ifdef USE_DAILY
  rainUpdateTotal = 0;
#endif
  for (int i = currentHour + 72; i > currentHour + 48; i--)
  {
    rainUpdateTotal += rainBucket[i];
    #ifdef MY_DEBUG
      //Serial.print(F("Adding rainBucket["));
      //Serial.print(i);
      //Serial.println(F("] to rainUpdateTotal."));
    #endif
  }
  #ifdef MY_DEBUG
    Serial.print(F("TX Day 4: rainUpdateTotal = "));
    Serial.println((float)rainUpdateTotal / 100.0);
  #endif
  send(msgRainVAR4.set((float)rainUpdateTotal / 100.0, 1));
  wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
#ifdef USE_DAILY
  rainUpdateTotal = 0;
#endif
  for (int i = currentHour + 96; i > currentHour + 72; i--)
  {
    rainUpdateTotal += rainBucket[i];
    #ifdef MY_DEBUG
      //Serial.print(F("Adding rainBucket["));
      //Serial.print(i);
      //Serial.println(F("] to rainUpdateTotal."));
    #endif
  }
  #ifdef MY_DEBUG
    Serial.print(F("TX Day 5: rainUpdateTotal = "));
    Serial.println((float)rainUpdateTotal / 100.0);
  #endif
  send(msgRainVAR5.set((float)rainUpdateTotal / 100.0, 1));
  wait(WAIT_BETWEEN_SEND, C_INTERNAL, I_TIME); // Wait before next message send
} // End transmitRainData

void receive(const MyMessage &message)
{

} // End receive

void receiveTime(unsigned long newTime)
{
  setTime(newTime + 30); // add time we wait for time arrival from controller

  char theTime[10];
  sprintf(theTime, "%d:%d:%d",hour(),minute(),second());
  Serial.println(F("Time received from controller..."));
  Serial.println(theTime);
  gotTime = true;
  currentHour = hour();
  if (lastHour == 0) {
    lastHour = hour();
  }
} // End receiveTime
