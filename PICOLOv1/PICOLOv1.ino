/*
_____________________________________________________________
Code for the PICOLO Flight Computer
Code by: Radhakrishna Vojjala
Modified by: Drew Miler
Date of last modification: 10 Apr 2025
Modified to accomate 2 PID control systems using gyroscope and photoresistor data
PID code reference: https://www.robotsforroboticists.com/pid-control/

Version 1.0
_____________________________________________________________

*/

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BNO055.h> 
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MS5611.h>   // Download from: https://github.com/jarzebski/Arduino-MS5611
#include <Adafruit_VEML7700.h>
#include "StemmaQtOLED.h"
// Additional libraries here
#include <utility/imumaths.h>
#include <Servo.h>

#include "variables.h"

#define GPS_RUN_RATE    2.0 // Max GPS update speed in Hz. May not update at this speed.
#define DATA_RATE 10000 // Max rate of data aqusition in Hz. Set to 100 or some huge number to remove the limiter
#define VERSION "1.0"



// Constants
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define SERVO_PIN (7)  // GPIO pin for servo on Pico
#define TARGET_ORIENTATION (0.0)  // Target heading in degrees
#define TOLERANCE (5.0)  // Tolerance in degrees
#define MAX_SERVO_SPEED (100)  // Maximum servo speed (0-180)
#define MIN_SERVO_SPEED (-100)    // Minimum servo speed (0-180)
#define NEUTRAL_SERVO (0)     // Neutral position (no rotation)

// PID Constants
#define KP (.8)  // Proportional gain
#define KI (0.1) // Integral gain  
#define KD (0.05)  // Derivative gain

#define photoKP (.15)
#define photoKI (0.015)
#define photoKD (0.015)

#define torqueKP (0.5)

// Config variables.

bool usingM8N = true; // true for M8N, false for M9N

// File header. Edit to add columns for other sensors.

String header = "Mode,ServoCommand,photo1,photo2,photo3,hh:mm:ss,T(min),T(s),T(ms),Hz,Fix Type,PVT,Sats,Date,Time,Lat,Lon,Alt(Ft),Alt(M),HorizAccuracy(MM),VertAccuracy(MM),VertVel(Ft/S),VertVel(M/S),ECEFstat,ECEFX(M),ECEFY(M),ECEFZ(M),NedVelNorth(M/S),NedVelEast(M/S),NedVelDown(M/S),GndSpd(M/S),Head(Deg),PDOP,ExtT(F),ExtT(C),IntT(F),IntT(C),Pa,kPa,ATM,PSI,MSTemp(C),MSTemp(F),Alt SL Ft,Alt SL M,Alt Rel Ft,Alt Rel M,VertVel(ft/s),VertVel(m/s),Accel(x),Accel(y),Accel(z),Deg/S(x),Deg/S(y),Deg/S(z),Ori(x),Ori(y),Ori(z),Mag_T(x),Mag_T(y),Mag_T(z)z,Version:" + String(VERSION);

void setup() {
  systemSetup();
}

void loop() {

  if ((millis() - nowTimeMS) >= loopTime) {
    systemUpdate();

    // assembling the data srting;

    data = "";
    OLEDstr = "";
    
    data.concat(mode);
    data.concat(",");
    OLEDstr.concat("Mode: " + String(mode) + "\n");
    data.concat(servoCommand);
    data.concat(",");
    OLEDstr.concat("ServoCMD: " + String(servoCommand) + "\n");
    data.concat(photo1);
    data.concat(",");
    OLEDstr.concat("Photo1: " + String(photo1) + "\n");
    data.concat(photo2);
    data.concat(",");
    OLEDstr.concat("Photo2: " + String(photo2) + "\n");
    data.concat(photo3);
    data.concat(",");
    OLEDstr.concat("Photo3: " + String(photo3) + "\n");
    data.concat(HHMMSS);
    data.concat(",");
    data.concat(String(nowTimeMin));
    data.concat(",");
    data.concat(String(nowTimeS));
    data.concat(",");
    data.concat(String(nowTimeMS));
    data.concat(",");
    data.concat(String(freq));
    data.concat(",");
    data.concat(fixTypeGPS);
    data.concat(",");
    data.concat(String(pvtStatus));
    data.concat(",");
    data.concat(String(SIV));
    data.concat(",");
    data.concat(String(gpsMonth));
    data.concat("/");
    data.concat(String(gpsDay));
    data.concat("/");
    data.concat(String(gpsYear));
    data.concat(",");
    data.concat(String(gpsHour));
    data.concat(":");
    data.concat(String(gpsMinute));
    data.concat(":");
    data.concat(String(gpsSecond));
    data.concat(".");
    

    if (gpsMillisecond < 10) {
      data.concat("00");
      data.concat(String(gpsMillisecond));
      data.concat(",");
    }
    else if (gpsMillisecond < 100) {
      data.concat("0");
      data.concat(String(gpsMillisecond));
      data.concat(",");
    }
    else{
      data.concat(String(gpsMillisecond)); 
      data.concat(",");
    }

    char paddedNumber[8]; // Buffer to hold the padded number (7 digits + null terminator)
    data.concat(String(gpsLatInt));
    data.concat(".");
    // Format the number with padded zeros using sprintf()
    sprintf(paddedNumber, "%07ld", gpsLatDec);
    data.concat(String(paddedNumber)); // Pad the number with zeros up to 7 digits
    data.concat(",");
    //OLEDstr.concat("Lat: " + String(gpsLatInt) + "." + String(paddedNumber) + "\n");

    data.concat(String(gpsLonInt)); 
    data.concat(".");
    // Format the number with padded zeros using sprintf()
    sprintf(paddedNumber, "%07ld", gpsLonDec);
    data.concat(String(paddedNumber)); // Pad the number with zeros up to 7 digits
    data.concat(",");
    //OLEDstr.concat("Lon: " + String(gpsLonInt) + "." + String(paddedNumber) + "\n");

    data.concat(String(gpsAltFt));
    data.concat(",");
    //OLEDstr.concat("GPSft: " + String(gpsAltFt) + "\n");
    data.concat(String(gpsAltM));
    data.concat(",");
    data.concat(String(gpsHorizAcc));
    data.concat(",");
    data.concat(String(gpsVertAcc));
    data.concat(",");
    data.concat(String(gpsVertVelFt));
    data.concat(",");
    data.concat(String(gpsVertVelM));
    data.concat(",");
    data.concat(String(ecefStatus));
    data.concat(",");
    data.concat(String(ecefX));
    data.concat(",");
    data.concat(String(ecefY)); 
    data.concat(",");
    data.concat(String(ecefZ));
    data.concat(","); 
    data.concat(String(velocityNED[0]));
    data.concat(",");
    data.concat(String(velocityNED[1])); 
    data.concat(",");
    data.concat(String(velocityNED[2]));
    data.concat(","); 
    data.concat(String(gpsGndSpeed));
    data.concat(",");
    data.concat(String(gpsHeading));
    data.concat(",");
    data.concat(String(gpsPDOP));
    data.concat(",");
    data.concat(String(outTempF));
    data.concat(",");
    data.concat(String(outTempC));
    data.concat(",");
    data.concat(String(inTempF));
    data.concat(",");
    data.concat(String(inTempC));
    data.concat(",");
    data.concat(String(pressPa));
    data.concat(",");
    data.concat(String(presskPa));
    data.concat(",");
    data.concat(String(pressATM));
    data.concat(",");
    data.concat(String(pressPSI));
    data.concat(",");
    data.concat(String(MStempC));
    data.concat(",");
    data.concat(String(MStempF));
    data.concat(",");
    data.concat(String(absAltFt));
    data.concat(",");
    //OLEDstr.concat("MSft: " + String(absAltFt) + "\n");
    data.concat(String(absAltM));
    data.concat(",");
    data.concat(String(relAltFt));
    data.concat(",");
    data.concat(String(relAltM));
    data.concat(",");
    data.concat(String(vertVelFt));
    data.concat(",");
    data.concat(String(vertVelM));
    data.concat(",");
    data.concat(String(accelerometer[0]));
    data.concat(",");
    data.concat(String(accelerometer[1]));
    data.concat(",");
    data.concat(String(accelerometer[2]));
    data.concat(",");
    data.concat(String(gyroscope[0]));
    data.concat(",");
    data.concat(String(gyroscope[1]));
    data.concat(",");
    data.concat(String(gyroscope[2]));
    data.concat(",");
    data.concat(String(orientation[2]));
    data.concat(",");
    data.concat(String(orientation[1]));
    data.concat(",");
    data.concat(String(orientation[0]));
    data.concat(",");
    OLEDstr.concat("Z: " + String(orientation[0]) + "\n");
    data.concat(String(magnetometer[0]));
    data.concat(",");
    data.concat(String(magnetometer[1]));
    data.concat(",");
    data.concat(String(magnetometer[2]));
    data.concat(",");

    /*
      data form additional sensors
    */

    Serial.println(data);
    SDstatus = logData(data, dataFilename);
    
    if (!SDstatus) {      
      digitalWrite(ERR_LED_PIN, HIGH);
      Serial.println("SD failed!");
    }

    //OLEDstr.concat("Sats: " + String(SIV) + "  Hz: " + String(freq) + "\n");
    //OLEDstr.concat("Hz:" + String(freq) + "\n");
    OLEDstr.concat("TimeMin: " + String(nowTimeMin) + "\n");
    //OLEDstr.concat("Ext: " + String(outTempF) + " F\nInt: " + String(inTempF) + " F\nMS: " + String(MStempF) + " F");
    printOLED(OLEDstr);

    prevTime = nowTimeMS;
  }
}
