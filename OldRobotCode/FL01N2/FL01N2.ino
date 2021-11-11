// FL01N3 - Human Interface for Flight Leader
// 13 Aug 2017: Initial development
// 13 Aug 2017 Rev 0:  read multikey input (including negative numbers); display on LCD
#include <Keypad.h>
#include "Arduino.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns
char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};
byte rowPins[ROWS] = {8, 7, 6, 5}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {4, 3, 2}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

//+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+
//     DEBUG SW     DEBUG SW     DEBUG SW     DEBUG SW     DEBUG SW     DEBUG SW
//+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+
const bool debug = false;   // bool false is zero; anything else is true
//const bool debug = true;

int inNum = 0;    // Keypad input
int sign = +1;    // sign of input number; set to negative if '*' pressed

//==========================================================================
//     SETUP     SETUP     SETUP     SETUP     SETUP     SETUP     SETUP
//==========================================================================
void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("System Ready");
  lcd.setCursor(0, 1);
}

//==========================================================================
//     LOOP     LOOP     LOOP     LOOP     LOOP     LOOP     LOOP     LOOP
//==========================================================================
void loop() {
  char key = keypad.getKey();

  if (key) {
    if (debug) {
      Serial.println(key);
    }

    lcd.print(key);

    if (key >= '0'  && key <= '9') {
      int someInt = key - '0';
      inNum = inNum * 10 + someInt;  // create a number from the character
      if (debug) {
        Serial.print("inNum= ");
        Serial.println(inNum);
      }
    }
    else {
      if (key == '*') {
        sign = -sign;
      }
      if (key  == '#') {
        lcd.setCursor(0, 1);
        lcd.print("Input: ");
        lcd.print(inNum * sign);
      }
    }
  }
}
