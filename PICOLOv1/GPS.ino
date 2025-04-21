/*
_____________________________________________________________
Code for M8N and M9N GPS running on UART.
Code originally by: Ashton Posey
Code modified for Pi Pico W by: Radhakrishna Vojjala
Date of last modification: 18 Feb 2025
_____________________________________________________________
*/

// Setup function for GPS

void GPSsetup() {

  gpsStatus = false;

  if (usingM8N) {
    GPStype = "M8N";
  }
  else {
    GPStype = "M9N";
    gpsBaud = 38400;
  }

  Serial.println("Checking GPS...");
  printOLED("Checking GPS...", true);

  for(int i = 0; (i < 20) && (!gpsStatus); i++){
    Serial1.begin(gpsBaud);
    gpsStatus = (sparkFunGNSS.begin(Serial1)) ? true : false;
  }

  if (gpsStatus){
    Serial.println("GPS Online!");
    printOLED("GPS Online!", true);
  }
  else {
    Serial.println("GPS Offline! Check wiring.");
    printOLED("GPS Offline!\nCheck wiring.", true);
    digitalWrite(ERR_LED_PIN, HIGH);
    delay(500);
    digitalWrite(ERR_LED_PIN, LOW);
    return;
  }
  
  Serial.println("Setting up GPS...");
  printOLED("Setting up GPS...");
      
  sparkFunGNSS.setUART1Output(COM_TYPE_NMEA | COM_TYPE_UBX); //Turn off UBX and RTCM sentences on the UART1 interface
  sparkFunGNSS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1); // Several of these are on by default on ublox board so let's disable them
  sparkFunGNSS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
  sparkFunGNSS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
  sparkFunGNSS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
  sparkFunGNSS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
  sparkFunGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1); // Leave only GGA enabled at current navigation rate
  
  sparkFunGNSS.setNavigationFrequency(GPS_RUN_RATE); //Set output rate
  byte rate = sparkFunGNSS.getNavigationFrequency(); //Get the update rate of this module
  Serial.println("Current Frequency: " + String(rate));
  printOLED("Setting up GPS...\nCurrent Frequency: " + String(rate));
  if(rate != GPS_RUN_RATE){
    for(int i=0; i<50; i++){ //reference list of dynamic models can be found at the bottom of this tab. 0 is pedestrian (default), 7 is airborne2g, 8 is airborne4g
      sparkFunGNSS.setNavigationFrequency(GPS_RUN_RATE); //Set output rate
      rate = sparkFunGNSS.getNavigationFrequency(); //Get the update rate of this module
      Serial.println("Current Frequency: " + String(rate));
      printOLED("Setting up GPS...\nCurrent Frequency: " + String(rate));
      if(rate == GPS_RUN_RATE) break;
    }
  }
  int dataDelay = 1000 / GPS_RUN_RATE;
  Serial.println("Data Delay = " + String(dataDelay));
  printOLED("Setting up GPS...\nData Delay = " + String(rate));
  int mesRate = sparkFunGNSS.setMeasurementRate(dataDelay); //Produce a measurement every 1000ms
  Serial.println("Current Navigation Rate: " + String(rate));
  printOLED("Setting up GPS...\nCurrent Nav Rate: " + String(rate));
  if(mesRate != dataDelay){
    for(int i=0; i<50; i++){ //reference list of dynamic models can be found at the bottom of this tab. 0 is pedestrian (default), 7 is airborne2g, 8 is airborne4g
      sparkFunGNSS.setMeasurementRate(dataDelay); //Set output rate
      mesRate = sparkFunGNSS.getMeasurementRate(); //Get the update rate of this module
      Serial.println("Current Navigation Rate: " + String(mesRate));
      printOLED("Setting up GPS...\nCurrent Nav Rate: " + String(mesRate));
      if(mesRate == dataDelay) break;
    }
  }
      
  for (int i=0; i<10; i++) sparkFunGNSS.setAutoPVTrate(1); //Tell the GNSS to send the PVT solution every measurement

  for(int i=0; i<50; i++){ //reference list of dynamic models can be found at the bottom of this tab. 0 is pedestrian (default), 7 is airborne2g, 8 is airborne4g
    sparkFunGNSS.setDynamicModel(DYN_MODEL); //if returns 255 then that is an error, unable to communicate with gps
    gpsStatus  = (int)sparkFunGNSS.getDynamicModel();
    Serial.println("Current Airmode: " + String(gpsStatus) + ", attempting to switch to airborne2g/7");
    printOLED("Setting up GPS...\nCurrent Airmode:\n" + String(gpsStatus) + "\n\nAttempting to switch\nto airborne2g/7");
    if(gpsStatus == 7) break;

  }

} 

// Update function for GPS

void GPSupdate(){

  if (!gpsStatus){
    return;
  }

  pvtStatus = sparkFunGNSS.getPVT();
  if(pvtStatus){
    gpsAltM =  sparkFunGNSS.getAltitude() * pow(10.0, -3); 
    gpsAltFt = gpsAltM * 3.280839895;
    gpsLat = sparkFunGNSS.getLatitude() * pow(10.0, -7);
        
    gpsLatDec = abs(long(gpsLat * pow(10.0, 7))) % 10000000;
    if (gpsLat < 0) gpsLatInt = floor(gpsLat) + 1;
    else gpsLatInt = floor(gpsLat);
    
    gpsLon = sparkFunGNSS.getLongitude() * pow(10.0, -7);
    gpsLonDec = abs(long(gpsLon * pow(10.0, 7))) % 10000000;
    if (gpsLon < 0) gpsLonInt = floor(gpsLon) + 1;
    else gpsLonInt = floor(gpsLon);

    SIV = sparkFunGNSS.getSIV();
        
    gpsMonth = sparkFunGNSS.getMonth();
    gpsDay = sparkFunGNSS.getDay();
    gpsYear = sparkFunGNSS.getYear();
    gpsHour = sparkFunGNSS.getHour() - 5;  // Their time zone is 5h ahead
    if (gpsHour < 0) gpsHour = gpsHour + 24;
    gpsMinute = sparkFunGNSS.getMinute();
    gpsSecond = sparkFunGNSS.getSecond();
    gpsMillisecond = sparkFunGNSS.getMillisecond();

    gpsTimeOfWeek = sparkFunGNSS.getTimeOfWeek();

    gpsGndSpeed = sparkFunGNSS.getGroundSpeed() * pow(10.0, -3); // mm to m
    gpsHeading = sparkFunGNSS.getHeading() * pow(10.0, -5); // deg * 10^5 to deg
    gpsPDOP = sparkFunGNSS.getPDOP();

    byte fixType = sparkFunGNSS.getFixType();
    if(fixType == 0) fixTypeGPS = "No fix";
    else if(fixType == 1) fixTypeGPS = "Dead reckoning";
    else if(fixType == 2) fixTypeGPS = "2D";
    else if(fixType == 3) fixTypeGPS =  "3D";
    else if(fixType == 4) fixTypeGPS = "GNSS + Dead reckoning";
    else if(fixType == 5) fixTypeGPS = "Time only";

    velocityNED[0] = sparkFunGNSS.getNedNorthVel() / 1000.0;
    velocityNED[1] = sparkFunGNSS.getNedEastVel() / 1000.0;
    velocityNED[2] = sparkFunGNSS.getNedDownVel() / 1000.0;

    gpsHorizAcc = sparkFunGNSS.getHorizontalAccEst(); // Print the horizontal accuracy estimate
    gpsVertAcc = sparkFunGNSS.getVerticalAccEst();

    updateUTM();
    sparkFunGNSS.flushPVT();

    gpsVertVelFt = (gpsAltFt - gpsPrevAlt) / ((millis() / 1000) - gpsPrevTime);
    gpsVertVelM = gpsVertVelFt * 0.3047999995367;
    gpsPrevAlt = gpsAltFt;
    gpsPrevTime = millis()/1000;
  }

  ecefStatus = sparkFunGNSS.getNAVPOSECEF();
  if (ecefStatus){

    ecefX = sparkFunGNSS.packetUBXNAVPOSECEF->data.ecefX; // convert ecefX to cm
    ecefY = sparkFunGNSS.packetUBXNAVPOSECEF->data.ecefY; // convert ecefY to cm
    ecefZ = sparkFunGNSS.packetUBXNAVPOSECEF->data.ecefZ; // convert ecefY to cm
    sparkFunGNSS.flushNAVPOSECEF(); //Mark all the data as read/stale so we get fresh data next time
  }
}

// Other GPS functions

void updateUTM() {//call function to find the UTM variables. the lat and long variabels that are being passed in are taken directally from the PVT values
  LLtoUTM(gpsLat, gpsLon, UTMNorthing, UTMEasting, UTMZoneNum); //conversion is using direct LLA values
  UTMZoneLetter = UTMLetterDesignator(gpsLat); //passes in current latitude in order to find the letter designation for northing and easting
}

//%%%%%%%%%%%%%%%%%%% BELOW ARE FUNCTIONS TO CONVERT FROM LLA TO UTM COORDINATES %%%%%%%%%%%%%%%%%%%
//this is a function that will convert a given LLA Value to UTM (Easting and Northing) coordinates.
// taken from:  https://github.com/bakercp/ofxGeo/blob/master/libs/UTM/include/UTM/UTM.h
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
// Grid granularity for rounding UTM coordinates to generate MapXY.
const double grid_size = 100000.0;    ///< 100 km grid

// WGS84 Parameters
#define WGS84_A    6378137.0   ///< major axis
#define WGS84_B   6356752.31424518  ///< minor axis
#define WGS84_F   0.0033528107    ///< ellipsoid flattening
#define WGS84_E   0.0818191908    ///< first eccentricity
#define WGS84_EP  0.0820944379    ///< second eccentricity

// UTM Parameters
#define UTM_K0    0.9996      ///< scale factor
#define UTM_FE    500000.0    ///< false easting
#define UTM_FN_N  0.0           ///< false northing, northern hemisphere
#define UTM_FN_S  10000000.0    ///< false northing, southern hemisphere
#define UTM_E2    (WGS84_E*WGS84_E) ///< e^2
#define UTM_E4    (UTM_E2*UTM_E2)   ///< e^4
#define UTM_E6    (UTM_E4*UTM_E2)   ///< e^6
#define UTM_EP2   (UTM_E2/(1-UTM_E2)) ///< e'^2

/**
   Determine the correct UTM letter designator for the
   given latitude

   @returns 'Z' if latitude is outside the UTM limits of 84N to 80S

   Written by Chuck Gantz- chuck.gantz@globalstar.com
*/
static inline char UTMLetterDesignator(double Lat)
{
  char LetterDesignator;

  if     ((84 >= Lat) && (Lat >= 72))  LetterDesignator = 'X';
  else if ((72 > Lat) && (Lat >= 64))  LetterDesignator = 'W';
  else if ((64 > Lat) && (Lat >= 56))  LetterDesignator = 'V';
  else if ((56 > Lat) && (Lat >= 48))  LetterDesignator = 'U';
  else if ((48 > Lat) && (Lat >= 40))  LetterDesignator = 'T';
  else if ((40 > Lat) && (Lat >= 32))  LetterDesignator = 'S';
  else if ((32 > Lat) && (Lat >= 24))  LetterDesignator = 'R';
  else if ((24 > Lat) && (Lat >= 16))  LetterDesignator = 'Q';
  else if ((16 > Lat) && (Lat >= 8))   LetterDesignator = 'P';
  else if (( 8 > Lat) && (Lat >= 0))   LetterDesignator = 'N';
  else if (( 0 > Lat) && (Lat >= -8))  LetterDesignator = 'M';
  else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
  else if ((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
  else if ((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
  else if ((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
  else if ((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
  else if ((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
  else if ((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
  else if ((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
  else if ((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
  // 'Z' is an error flag, the Latitude is outside the UTM limits
  else LetterDesignator = 'Z';
  return LetterDesignator;
}

/**
   Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532

   East Longitudes are positive, West longitudes are negative.
   North latitudes are positive, South latitudes are negative
   Lat and Long are in fractional degrees

   Written by Chuck Gantz- chuck.gantz@globalstar.com
*/
static inline void LLtoUTM(const double Lat, const double Long,
                           double &UTMNorthing, double &UTMEasting,
                           int &UTMZone) //this has been edited so it includes UTM as an int not a char
{
  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double k0 = UTM_K0;

  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;

  //Make sure the longitude is between -180.00 .. 179.9
  double LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180;
  // Serial.println("HERE IS LONG TEMP: " + String(LongTemp));

  double LatRad = Lat * DEG_TO_RAD;
  double LongRad = LongTemp * DEG_TO_RAD;
  double LongOriginRad;
  int    ZoneNumber;

  ZoneNumber = int((LongTemp + 180) / 6) + 1;

  if ( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
    ZoneNumber = 32;

  // Special zones for Svalbard
  if ( Lat >= 72.0 && Lat < 84.0 )
  {
    if (      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
    else if ( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
    else if ( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
    else if ( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
  }
  // +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;
  LongOriginRad = LongOrigin * DEG_TO_RAD;

  //compute the UTM Zone from the latitude and longitude
  // sprintf(UTMZone, "%d%c", ZoneNumber, UTMLetterDesignator(Lat)); //commented out bc doesnt work

  UTMZone = ZoneNumber;    //save the found zone number to UTMZone

  eccPrimeSquared = (eccSquared) / (1 - eccSquared);

  N = a / sqrt(1 - eccSquared * sin(LatRad) * sin(LatRad));
  T = tan(LatRad) * tan(LatRad);
  C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
  A = cos(LatRad) * (LongRad - LongOriginRad);

  M = a * ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64
          - 5 * eccSquared * eccSquared * eccSquared / 256) * LatRad
          - (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32
          + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(2 * LatRad)
          + (15 * eccSquared * eccSquared / 256
          + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(4 * LatRad)
          - (35 * eccSquared * eccSquared * eccSquared / 3072) * sin(6 * LatRad));

  UTMEasting = (double)
              (k0 * N * (A + (1 - T + C) * A * A * A / 6
              + (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A * A * A / 120)
              + 500000.0);

  UTMNorthing = (double)
              (k0 * (M + N * tan(LatRad)
              * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24
              + (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A * A * A * A * A * A / 720)));

  if (Lat < 0)
  {
    //10000000 meter offset for southern hemisphere
    UTMNorthing += 10000000.0;
  }
}

/**
   Converts UTM coords to lat/long.  Equations from USGS Bulletin 1532

   East Longitudes are positive, West longitudes are negative.
   North latitudes are positive, South latitudes are negative
   Lat and Long are in fractional degrees.

   Written by Chuck Gantz- chuck.gantz@globalstar.com
*/
static inline void UTMtoLL(const double UTMNorthing, const double UTMEasting, const char* UTMZone, double& Lat, double& Long){
  double k0 = UTM_K0;
  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double eccPrimeSquared;
  double e1 = (1 - sqrt(1 - eccSquared)) / (1 + sqrt(1 - eccSquared));
  double N1, T1, C1, R1, D, M;
  double LongOrigin;
  double mu, phi1Rad;
  double x, y;
  int ZoneNumber;
  char* ZoneLetter;

  x = UTMEasting - 500000.0; //remove 500,000 meter offset for longitude
  y = UTMNorthing;

  ZoneNumber = strtoul(UTMZone, &ZoneLetter, 10);
  if ((*ZoneLetter - 'N') < 0){//remove 10,000,000 meter offset used for southern hemisphere
      y -= 10000000.0;
  }

  //+3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;
  eccPrimeSquared = (eccSquared) / (1 - eccSquared);

  M = y / k0;
  mu = M / (a * (1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64
                 - 5 * eccSquared * eccSquared * eccSquared / 256));

  phi1Rad = mu + ((3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * mu)
                  + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * mu)
                  + (151 * e1 * e1 * e1 / 96) * sin(6 * mu));

  N1 = a / sqrt(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad));
  T1 = tan(phi1Rad) * tan(phi1Rad);
  C1 = eccPrimeSquared * cos(phi1Rad) * cos(phi1Rad);
  R1 = a * (1 - eccSquared) / pow(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad), 1.5);
  D = x / (N1 * k0);

  Lat = phi1Rad - ((N1 * tan(phi1Rad) / R1)
                  * (D * D / 2
                  - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * eccPrimeSquared) * D * D * D * D / 24
                  + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * eccPrimeSquared
                  - 3 * C1 * C1) * D * D * D * D * D * D / 720));

  Lat = Lat * RAD_TO_DEG;

  Long = ((D - (1 + 2 * T1 + C1) * D * D * D / 6
           + (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * eccPrimeSquared + 24 * T1 * T1)
           * D * D * D * D * D / 120)
           / cos(phi1Rad));
  Long = LongOrigin + Long * RAD_TO_DEG;

}
