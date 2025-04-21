/*
_____________________________________________________________
Code for the entire system of the Pi Pico W.
Code by: Radhakrishna Vojjala
Date of last modification: 20 Mar 2025
_____________________________________________________________
This file contains the Setup and Update functions for the entire system. The code is meant to be called in setup() and loop().
*/

// System wide setup function.

void systemSetup() {

  Serial.begin(SERIAL_BAUD);

  Wire.begin(); // default I2C clock
  Wire1.setSCL(I2C_1_SCL);
  Wire1.setSDA(I2C_1_SDA);
  Wire1.begin(); // default I2C clock

  beginOLED(); 

  Serial.println("Initialising....");
  printOLED("Initialising....", true);
  pinMode(ERR_LED_PIN, OUTPUT); // red LED
  pinMode(LOOP_LED_PIN, OUTPUT); // yellow LED
  pinMode(redledside, OUTPUT);
  pinMode(redledtop, OUTPUT);
  pinMode(blueledside, OUTPUT);
  pinMode(blueledtop, OUTPUT);
  // LED test
  digitalWrite(ERR_LED_PIN, HIGH);
  delay(100);
  digitalWrite(LOOP_LED_PIN, HIGH);
  delay(100);
  digitalWrite(ERR_LED_PIN, LOW);
  delay(100);
  digitalWrite(LOOP_LED_PIN, LOW);

  if (bno.begin()){
    Serial.println("BNO Online!");
    printOLED("BNO Online!", true);
  }
  else {
    Serial.println("BNO Offline! Check wiring.");
    printOLED("BNO Offline!\nCheck wiring.", true);
    digitalWrite(ERR_LED_PIN, HIGH);
    delay(500);
    digitalWrite(ERR_LED_PIN, LOW);
  }


  if (MSsetup()){
    Serial.println("MS5611 Online!");
    printOLED("MS5611 Online!", true);
  }
  else {
    Serial.println("MS5611 Offline! Check wiring.");
    printOLED("MS5611 Offline!\nCheck wiring.", true);
    digitalWrite(ERR_LED_PIN, HIGH);
    delay(500);
    digitalWrite(ERR_LED_PIN, LOW);
  }

  /*inStatus = inTher.begin(10);
  outStatus = outTher.begin(10);

  if (inStatus){
    Serial.println("Internal Thermistor Online!");
    printOLED("Internal Thermistor\nOnline!", true);
  }
  else {
    Serial.println("Internal Thermistor Offline. Check wiring.");
    printOLED("Internal Thermistor\nOffline.\nCheck wiring.", true);
    digitalWrite(ERR_LED_PIN, HIGH);
    delay(500);
    digitalWrite(ERR_LED_PIN, LOW);
  }
  
  if (outStatus){
    Serial.println("External Thermistor Online!");
    printOLED("External Thermistor\nOnline!", true);
  }
  else {
    Serial.println("External Thermistor Offline. Check wiring.");
    printOLED("External Thermistor\nOffline.\nCheck wiring.", true);
    digitalWrite(ERR_LED_PIN, HIGH);
    delay(500);
    digitalWrite(ERR_LED_PIN, LOW);
  }
  */
  //GPSsetup();
  photoresistorsetup();

  /*
     Add setup code for additional sensors here
  */

    // == OPTIONAL =====
    // Can set non-default gain and integration time to
    // adjust for different lighting conditions.
    // ================

  SDsetup(dataFilename, dataFileN1, dataFileN2);
  logData(header, dataFilename);

  loopTime = 1000 / DATA_RATE;

  Controlwheelsetup();
  Serial.println("Setup Finished");
  printOLED("Setup Finished", true);
  Serial.println(header);
}

// System wide update function. Updates all sensors.

void systemUpdate(){

  // updating timers

  nowTimeMS = millis();
  digitalWrite(LOOP_LED_PIN, LOW);
  nowTimeS = nowTimeMS / 1000.0;
  nowTimeMin = nowTimeS / 60;
  freq = 1.0/((nowTimeMS-prevTime)/1000.0);
  HHMMSS = timeToHhmmss(nowTimeMS);

  // updating sensors

  //GPSupdate();
  MSupdate();
  BNOupdate();

  //inTher.update();
  //inTempF = inTher.getTempF();
  //inTempC = inTher.getTempC();
  //outTher.update();
  //outTempF = outTher.getTempF();
  //outTempC = outTher.getTempC();


  //Addtional sensor update code here.

  //ReactionControlWheelCode

  mode = "Idle"; //sets system mode to idle
  servoCommand = 0; //sets initial servo speed to 0
  redLEDon(); 
  blueLEDon(); // turns both LEDS on to indicate idle state

 

  if (nowTimeMin > 2.25 && nowTimeMin < 3){ // Puts the gryoscope PID system in control of the servo only at defined time intervals
    mode = "Gryo"; // sets system mode to Gyro
     // Get current orientation from IMU
    sensors_event_t event;
    bno.getEvent(&event);
    float currentHeading = event.orientation.x;  // Using x-axis for heading
    // Calculate error (difference from target orientation)
    float error = calculateHeadingError(currentHeading, TARGET_ORIENTATION);
    // Calculate PID output
    float controlOutput = calculatePID(error);
    torque = accelerometer[2] * torqueKP;
    // Convert PID output to servo command
    servoCommand = mapPIDToServo(controlOutput); //function that maps the calculated PID output to servo control
    servoCommand = servoCommand + torque;

    redLEDoff();
  } 
  
  // Delay for next reading
  delay(BNO055_SAMPLERATE_DELAY_MS); // optional delay that decreases system Hz but reduces gyro drift

  //Photoresistor data collection code (using a MUX)
  digitalWrite(s0,LOW);
  digitalWrite(s1,LOW);
  digitalWrite(s2,LOW);
  digitalWrite(s3,LOW);
  photo1 = analogRead(muxSig); 

  digitalWrite(s0,HIGH);
  digitalWrite(s1,LOW);
  digitalWrite(s2,LOW);
  digitalWrite(s3,LOW);
  photo2 = analogRead(muxSig);

  digitalWrite(s0,LOW);
  digitalWrite(s1,HIGH);
  digitalWrite(s2,LOW);
  digitalWrite(s3,LOW);
  photo3 = analogRead(muxSig);

  diffp1p2 = abs(photo1 - photo2);

  // Photoresistor control system
  if (nowTimeMin > .5 && nowTimeMin < 2){ //the photoresistors are only in control at set time intervals
    //photo3 should be facing the sun while photo 1 and 2 are pointing 90* left and right from the sun
    mode = "Photo";
    n = 0;
    blueLEDoff();
    while (n == 0) {
      n = 1;
      if (diffp1p2 < photomargin) { // Checks if photo1 and 2 are basically equal
        if (photo3 > photo1 && photo3 > photo2) { // if so, checks to make sure that photo 3 is facing the sun and is not facing away (180* off)
          n = 1; //exits the while loop
        } else { //the box is flipped 180*, so the box needs to turn around
          servoCommand = 100; //sets servo spin to max
          n = 1; // exits the while loop
        }
      }
      if ((photo2 - photo1) > photomargin) { //If photo2 is larger than photo1, the servo spins to rotate the box back
        servoCommand = 50; //currently working on implemmenting a PID system for the photoresistors, but right now the servo is set to a base speed of 50
        n = 1; 
      }
      if ((photo1 - photo2) > photomargin) { //If photo 1 is larger than photo2, the servo spins in the opposite direction to rotate the box
        servoCommand = -50;
        n = 1;
      }
    }
  }
  setServoSpeed(servoCommand); //the servo is commanded to move whatever was previously calculated
}

// Function to convert timer to HHMMSS format 

String timeToHhmmss(int milli) {

  int timerS = milli / 1000;
  int hours = timerS / 3600;
  int hoursRem = timerS % 3600;
  int mins = hoursRem / 60;
  int secs = hoursRem % 60;
  char timeStr[20];
  sprintf(timeStr, "%02d:%02d:%02d", hours, mins, secs);
  return String(timeStr);
}