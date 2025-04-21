/*
_____________________________________________________________
Code for MS5611 connected via I2C to Pi Pico W.
Code by: Radhakrishna Vojjala
Date of last modification: 7 Jan 2025
_____________________________________________________________
This file contains the Setup and Update functions for the MS5611 pressure sensor.
*/

// MS setup function;

bool MSsetup() {

  msOnline = ms.begin(MS5611_ULTRA_HIGH_RES); // ULTRA HIGH RES is a higher prescision but slower way of running the MS 
  refPress = ms.readPressure(); // This pressure will be used as reference to calculate altitude relative to startup location.

  return msOnline; 
}

// MS update function

void MSupdate() {

  double prevM = absAltM;
  double prevFt = absAltFt;
  MStempC = ms.readTemperature();
  MStempF = (MStempC * 1.8) + 32;
  pressPa = ms.readPressure();
  presskPa = pressPa / 1000.0;
  pressATM = pressPa / 101325.0;
  pressPSI = pressPa / 6894.75729317;
  absAltM = ms.getAltitude(pressPa);
  absAltFt = absAltM * 3.28083989501;
  relAltM = ms.getAltitude(pressPa, refPress);
  relAltFt = relAltM * 3.28083989501;
  vertVelM = ((absAltM - prevM) * 1000.0)/(nowTimeMS - prevTime);
  vertVelFt = ((absAltFt - prevFt) * 1000.0)/(nowTimeMS - prevTime);

}