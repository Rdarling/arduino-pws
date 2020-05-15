/*
  Arduino Weather Station
  Modified by: Ronald Darling January 19,2020 - RWD my initials
  the lines or lines with RWD at the end indicate my modifications
  The sparkfun weather shield used is DEV-13956
  this weather shield uses Si7021 humidity/tempurature sensor
  MPL3115A2 Barometric sensor and the ALS-PT19 light sensor
  Also a GPS module GP-735 is installed.
  ;;
  By: Radek Kaczorek, December 19th, 2016
  Based on example by: Nathan Seidle, SparkFun Electronics, November 16th, 2013
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Much of this is based on Mike Grusin's USB Weather Board code: https://www.sparkfun.com/products/10586

  This code reads all the various sensors (wind speed, direction, rain gauge, humidty, pressure, light, batt_lvl, gps latitude, gps longitude, gps altitude, sat no, gps date, gps time)
  and reports it over the serial com port.

  This code assumes the GP-635T (GP-735T - RWD) GPS module is attached
*/

#include <Wire.h> //I2C needed for sensors
#include "SparkFunMPL3115A2.h" //Pressure sensor
#include "SparkFun_Si7021_Breakout_Library.h" //Humidity/temperature sensor
#include <SoftwareSerial.h> //Needed for GPS and WiFi
#include <TinyGPS++.h> //GPS parsing
#include <math.h>

// GPS
TinyGPSPlus gps;
static const int RXPin = 5, TXPin = 4; //GPS is attached to pin 4(TX from GPS) and pin 5(RX into GPS)
SoftwareSerial gpsSerial(RXPin, TXPin);

MPL3115A2 myPressure; //Create an instance of the pressure sensor
Weather myHumidity; //Create an instance of the humidity sensor

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// digital I/O pins
const byte WSPEED = 3;
const byte RAIN = 2;
const byte STAT1 = 8; // Green status LED - weather data
const byte STAT2 = 7; // Blue status LED - GPS fix
const byte GPS_PWRCTL = 6; //Pulling this pin low puts GPS to sleep but maintains RTC and RAM

// analog I/O pins
const byte REFERENCE_3V3 = A3;
const byte LIGHT = A1;
const byte BATT = A2;
const byte WDIR = A0;

// update time in seconds
const byte UPDATE = 30;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastSecond = 0; //The millis counter to see when a second rolls by
byte seconds = 0;

long lastWindCheck = 0;

// volatiles are subject to modification by IRQs
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;
volatile float rainFall = 0;
volatile unsigned long raintime, rainlast, raininterval;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

  if (raininterval > 10) // ignore switch-bounce glitches less than 10ms after initial edge
  {
    //Each dump is 0.011" // 0.2794 mm of water   modified by RWD for inches
    rainFall += 0.011;     // += 0.2794; //Increase amount of rain  modified by RWD
    rainlast = raintime;  // set up for next event
  }
}

void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); //Grab the current time
    windClicks++; //There is 1.492MPH for each click per second = 1.492MPH * 1609.34/3600 = 0.666982022 m/s for each click per second
  }
}

void setup()
{
  Serial.begin(57600);   // was 115200 rwd 
  gpsSerial.begin(9600);

  pinMode(STAT1, OUTPUT); //Status LED Green - Weather data refresh
  pinMode(STAT2, OUTPUT); //Status LED Blue - GPS fix

  pinMode(GPS_PWRCTL, OUTPUT);
  digitalWrite(GPS_PWRCTL, HIGH); //Pulling this pin low puts GPS to sleep but maintains RTC and RAM

  pinMode(WSPEED, INPUT_PULLUP); // input from windspeed sensor
  pinMode(RAIN, INPUT_PULLUP); // input from rain gauge sensor

  pinMode(REFERENCE_3V3, INPUT);
  pinMode(LIGHT, INPUT);

  //Configure the pressure sensor
  // refer to the MPS311 library for more info
  myPressure.begin(); // Get sensor online
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  //myPressure.setModeAltimeter();
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags

  //Configure the humidity sensor
  myHumidity.begin();
  //myHumidity.heaterOff();
  //myHumidity.reset();
  
  lastSecond = millis();

  // attach external interrupt pins to IRQ functions
  attachInterrupt(0, rainIRQ, FALLING);
  attachInterrupt(1, wspeedIRQ, FALLING);

  // turn on interrupts
  interrupts();
  
  Serial.println("Arduino Personal Weather Station for Fahrenheit ");
}

void loop()
{
  //Keep track of which minute it is
  if (millis() - lastSecond >= 1000)
  {
    digitalWrite(STAT1, HIGH); //Green stat LED
    //Serial.println("turn green LED on");
    //&& gps.location.age() > 3600000
    if (get_battery_level() > 3.00 ) //Wake up GPS from sleep every hour if battery level is OK
    {
      //Serial.println("gps power up");
      digitalWrite(GPS_PWRCTL, HIGH);
    }

    if (gps.location.isUpdated())
    {
      //Serial.println("gps updating");
      digitalWrite(STAT2, LOW); //Turn off Blue stat LED if GPS is fixed
      //digitalWrite(GPS_PWRCTL, LOW); //Put GPS to sleep when fixed and wake up every hour
    } else {
      //Serial.println("gps power down");
      digitalWrite(STAT2, HIGH); //Turn on Blue stat LED if GPS is looking for fix
    }

    lastSecond += 1000;

    //Get all sensors readings every 10 seconds
    if (++seconds == UPDATE)
    {
      seconds = 0;
      getSensors();
      if (digitalRead(GPS_PWRCTL))
         getGPS(1000); //Wait 1 second, and gather GPS data
    }

    // Blink every loop
    digitalWrite(STAT1, LOW); //Turn off Green stat LED
    digitalWrite(STAT2, LOW); //Turn off Blue stat LED

  }
  
}

//While we delay for a given amount of time, gather GPS data
static void getGPS(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}


//Calculates each of the variables
void getSensors()
{
//  bool heaterstate = False;
  int winddir = 0; // [0-360 instantaneous wind direction]
  float windspeedmps = 0; // [m/s instantaneous wind speed]
  float humidity = 0; // [%]
  float MPLtempc = 0; // [temperature C] temp from MPL3115A2
  float Sitempc1 = 0;// get temp from Si7021
  float Sitempf1 = 0;//
  float Sitempc2 = 0;// read temp from Si7021
  float Sitempf2 = 0;//
  float MPLtempf = 0; // [temperature F] - RWD
  float Kpressure = 0;// reading from the sensor
  float pressure = 0; // kpa divided by 100 == pressure (Pa)
  float MasterTempF = 0;
  float MasterTempC = 0;
  float mmHg = 0;   // calculated bu UNO
  float inHg = 0;   // 1 kilopascal = 0.2953
  float mbar = 0;   // mbar calculated by arduino
  float MPLaltitude = 0;
  float rainmm = 0; // rain mm from last report
  float raininch = 0; // rain inch from report - RWD
  float dewptc; // [dewpoint C]
  float dewptf; // [dewpoint F]
  float batt_lvl = 11.8; //[analog value from 0 to 1023]
  float light_lvl = 455; //[analog value from 0 to 1023]

  //Calc winddir
  winddir = get_wind_direction();

  //Calc windspeed
  windspeedmps = get_wind_speed();

  //Calc humidity fom Si7021
  // humidity = myHumidity.readHumidity();
  humidity = myHumidity.getRH();
  
  
  Sitempf1 = myHumidity.getTempF();
  //Serial.print("Si7021 get tempf1 = ");
  //Serial.print(tempf1);
  Sitempc1 = myHumidity.getTemp();
  //Serial.print(" Si7021 get tempc1 = ");
  //Serial.print(tempc1);
  //
  Sitempf2 = myHumidity.readTempF();
  //Serial.print(" Si7021 read tempf2 = ");
  //Serial.print(tempf2);
  Sitempc2 = myHumidity.readTemp();
  //Serial.print(" Si7021 read tempc2 = ");
  //Serial.print(tempc2);
  //Serial.println();
  
  //Calc temp from pressure sensor
  MPLtempc = myPressure.readTemp(); //MPL3115A2
  //Serial.print(" MPL3115A2 tempc = ");
  //Serial.print(tempc);
  MPLtempf = myPressure.readTempF();
  //Serial.print(" MPL3115A2 tempf = ");
  //Serial.print(tempf);
  //Serial.println("  test end");
  //Serial.println();
  //tempf = (tempc * (9/5) + 32);   // convert tempc reading to Fahrenheit
  // Avereage all fahrenheit readings
  //MasterTempF = (Sitempf1 + Sitempf2 + MPLtempf)/3;  // average RWD
  // Average all celcius readings
  //MasterTempC = (Sitempc1 + Sitempc2 + MPLtempc)/3; // average  RWD
  MasterTempC = Sitempc1-4;
  //Serial.print("Get  ");
  //Serial.print(Sitempc1);
  //Serial.println();
  //Serial.print(" Average tempf = ");
  //Serial.print(MasterTempF);
  //Serial.print("read  ");
  //Serial.print(Sitempc2);
  //Serial.println();
  //Serial.print(" Average tempc = ");
  //Serial.print(MasterTempC);
  //Serial.println();
  
  //Calc rainmm and reset rainFall
  raininch = rainFall;      // changed from mm to inch
  rainFall = 0;

  //Calc pressure from the MPL3115A2
  Kpressure = myPressure.readPressure();   // pressure read from MLP in kPA
  pressure = Kpressure / 100;                        // pressure divided by 100
  mmHg = pressure * 0.00750062;           // convert pressure to mmHg
  //
  mbar = pressure / 100;                 // convert pressure to mbar

  inHg = Kpressure * 0.2953;             //   convert KPa to inhg
  //
  //pressure /= 100; //pressure is now in millibars
  int station_elevation_m = gps.altitude.meters();       //1638;
  
  float part1 = pressure - 0.3; //Part 1 of formula
  
  //Serial.print(station_elevation_m);
  const float part2 = 8.42288 / 100000.0;
  float part3 = pow((pressure - 0.3), 0.190284);
  float part4 = (float)station_elevation_m / part3;
  float part5 = (1.0 + (part2 * part4));
  float part6 = pow(part5, (1.0/0.190284));
  float altimeter_setting_pressure_mb = part1 * part6; //Output is now in adjusted millibars
  float baroin = altimeter_setting_pressure_mb * 0.02953;
  //Serial.print(" Inches of Mercury = ");
  //Serial.print(baroin);
  //Serial.println();
  //
  //Calc dewptc
  dewptc = pow(humidity/100,0.125) * (112 + (0.9 * Sitempc1)) + (0.1 * Sitempc1) - 112;
  dewptf = (dewptc * (9/5) + 32);
  //
  //MPLaltitude = myPressure.readAltitude();
  //Serial.print(" MPL3115A2 MPLaltitude = ");
  //Serial.print(MPLaltitude);
  //Serial.println();
  //Calc light level
  light_lvl = get_light_level();

  //Calc battery level
  batt_lvl = get_battery_level();

  // Print to serial
  Serial.print("$,windDir=");     // 0
  Serial.print(winddir);          // 1
  Serial.print(",windSpeed=");
  Serial.print(windspeedmps, 1);  // 2
  Serial.print(",outHumidity=");
  Serial.print(humidity, 2);      // 3
  Serial.print(",outTemp=");
  Serial.print(MasterTempC, 2);   // 4
  //Serial.print(",outTempC=");
  //Serial.print(MasterTempC, 2);   // 
  Serial.print(",rain=");
  Serial.print(raininch, 4);      // 5
  Serial.print(",pressure=");
  Serial.print(pressure, 2);      // 6
  //Serial.print(",inHg=");
  //Serial.print(baroin, 2);        // 8
  //Serial.print(",mbar =");
  //Serial.print(mbar, 2);          // 9
  //Serial.print(",DewPoint=");
  //Serial.print(dewptf, 2);      
  Serial.print(",light=");
  Serial.print(light_lvl, 2);     // 7
  Serial.print(",latitude=");
  Serial.print(gps.location.lat(), 6);    // 8
  Serial.print(",longitude=");
  Serial.print(gps.location.lng(), 6);    // 9
  Serial.print(",altitude=");
  Serial.print(gps.altitude.feet());      // 10
  //Serial.print(",AltitudeM=");
  //Serial.print(gps.altitude.meters());    // 14
  //Serial.print(gps.altitude.meters()*3.28084);
  // calc meter to foot
  // foot = meter * 3.28084
  Serial.print(",satellites=");           // 11
  Serial.print(gps.satellites.value());

  char sz[32];
  Serial.print(",fixDate=");
  sprintf(sz, "%02d/%02d/%02d", gps.date.month(),  gps.date.day(),  gps.date.year());  // 12
  Serial.print(sz);

  Serial.print(",fixTime=");
  sprintf(sz, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());// 13
  Serial.print(sz);

  Serial.print(",supplyVoltage=");
  Serial.print(batt_lvl, 2);    // 14

  Serial.print(",");            // 15
  Serial.println("#");
}

//Returns the voltage of the light sensor based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
float get_light_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);
  float lightSensor = analogRead(LIGHT);
  operatingVoltage = 3.3 / operatingVoltage; //The reference voltage is 3.3V
  lightSensor = operatingVoltage * lightSensor;
  return (lightSensor);
}

//Returns the voltage of the raw pin based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
//Battery level is connected to the RAW pin on Arduino and is fed through two 5% resistors:
//3.9K on the high side (R1), and 1K on the low side (R2)
float get_battery_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);
  float rawVoltage = analogRead(BATT);
  operatingVoltage = 3.30 / operatingVoltage; //The reference voltage is 3.3V
  rawVoltage = operatingVoltage * rawVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin
  
  //voltage divider on weather shield
  // rawVoltage *= 4.90; //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage

  // voltage divider MOD
  // 1) On Weather Shield: cut the track from original voltage divider to A2 pin (bootom layer)
  // 2) Solar Charger Shield: connect voltage divider to A2 pin (top layer - R6 soldering field from R5 resistor)
  
  //voltage divider on solar charger shield
  rawVoltage *= 1.74; // multiple BATT voltage by the voltage divider to get actual system voltage  
  
  return (rawVoltage);
}

//Returns the instataneous wind speed
float get_wind_speed()
{
  float deltaTime = millis() - lastWindCheck;
  deltaTime /= 1000.0; //Convert milliseconds to seconds
  float windSpeed = (float)windClicks / deltaTime;
  windClicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();
  windSpeed *= (1.492 * 1609.34 / 3600.0); //1.492 * 1609.34 / 3600 = 0.666982022 m/s
  return (windSpeed);
}

//Read the wind direction sensor, return heading in degrees
int get_wind_direction()
{
  unsigned int adc;
  adc = analogRead(WDIR); // get the current reading from the sensor
  //Serial.print(adc);
  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.
  if (adc < 380) return (113);
  if (adc < 393) return (68);
  if (adc < 414) return (90);
  if (adc < 456) return (158);
  if (adc < 508) return (135);
  if (adc < 551) return (203);
  if (adc < 615) return (180);
  if (adc < 680) return (23);
  if (adc < 746) return (45);
  if (adc < 801) return (248);
  if (adc < 833) return (225);
  if (adc < 878) return (338);
  if (adc < 913) return (0);
  if (adc < 940) return (293);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  return (-1); // error, disconnected?
}
