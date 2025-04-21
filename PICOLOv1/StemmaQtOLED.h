/*
_____________________________________________________________
Code for a 128 x 64 pixel StemmaQT OLED Screen connected via I2C.
Code by: Radhakrishna Vojjala
Date of last modification: 22 Jan 2025
_____________________________________________________________

This file contains the Setup, Clear and display functions for the StemmaQT OLED Screen.
*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SW 128
#define SH 64
#define OLED_ADD 0x3D

Adafruit_SSD1306 OLED(SW, SH, &Wire, -1);
unsigned long waitTime = 2000;  // Time in ms to hold each OLED message if wait is used.

void beginOLED() { 

  delay(500); //OLED needs a bit of time after bootup
  OLED.begin(SSD1306_SWITCHCAPVCC,OLED_ADD);
  OLED.display();
  delay(2000);
  OLED.clearDisplay();
  OLED.setTextSize(1);
  OLED.setTextColor(SSD1306_WHITE);      
  OLED.setCursor(0,0);
  OLED.clearDisplay();
}

void clearOLED() {

  OLED.display();
  OLED.clearDisplay();
  OLED.display();
  OLED.setCursor(0,0);
}

// Display function. Displays a string and you can set wait to true if you want the screen to hold the message for a bit so you can read it. 

void printOLED(String str, bool wait = false){

  OLED.clearDisplay();
  OLED.setCursor(0,0);
  OLED.print(str);
  OLED.display();
  if (wait) {

    delay(waitTime);
  }
}