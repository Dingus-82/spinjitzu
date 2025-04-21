/*
_____________________________________________________________
Code for SPI SD card reader for Pi Pico W
Code by: Radhakrishna Vojjala
Date of last modification: 6 Jan 2025
_____________________________________________________________
This file contains the Setup and Update functions for any SPI SD card reader.
*/

// SD setup Function

bool SDsetup(char *dataFile, byte const &N1, byte const &N2){

  pinMode(CS, OUTPUT);
  SPI.setRX(MISO);
  SPI.setTX(MOSI);
  SPI.setSCK(SCK);

  if (!SD.begin(CS)) {
    SDstatus = false;
    Serial.println("SD failed! Trying again...");
    printOLED("SD failed!\nTrying again...", true);
    for(int i = 0; i < 10; i++){

      if (SD.begin(CS)){
        SDstatus = true;
        break;
      }
      
    }
    if (!SD.begin(CS)){
      Serial.println("SD failed! Check wiring and SD.");
      printOLED("SD failed!\nCheck wiring and SD.", true);
      digitalWrite(ERR_LED_PIN, HIGH);
      delay(500);
      digitalWrite(ERR_LED_PIN, LOW);
    }
    else {
      SDstatus = true;
    }
  }

  if (SDstatus){

    for (byte i = 0; i < 100; i++) {

      dataFile[N1] = '0' + i/10; 
      dataFile[N2] = '0' + i%10;
      if (!SD.exists(dataFile)) {

        dataLog = SD.open(dataFile, FILE_WRITE);
        dataLog.close();
        Serial.println("SD works, logging to: " + String(dataFile));
        printOLED("SD works,\nlogging to:\n" + String(dataFile), true);
        SDfull = false;
        break;
      }
      
    }

    if(SDfull){
      
      Serial.println("Not enough file names, Clear SD.");
      printOLED("Not enough file\nnames, Clear SD.");
      SDstatus = false;
    }
  }

  return SDstatus;
}

// SD logging Function

bool logData(String const &Data, char const *dataFile){

  dataLog = SD.open(dataFile, FILE_WRITE);
  if (!dataLog) {
    return false;
  }
  dataLog.seek(EOF);
  dataLog.println(Data);
  dataLog.close();

  return true;
}