//SC00N2 -- Human Interface for Swarm Controller
// 6 Aug 2017: Initial development
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
int xi = 750;
int yi = 1004;
int xo = 749;
int yo = 749;
String data = "";
bool toggle = false;    //if false don't display
//==========================================================================
//     SETUP     SETUP     SETUP     SETUP     SETUP     SETUP     SETUP
//==========================================================================
void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("System Ready");

  // Start the I2C Bus as Slave on address 9
  Wire.begin(9);

  /*
    lcd.print("IN:x.");
    lcd.print(xi);
    lcd.print(" y.");
    lcd.print(yi);
    lcd.setCursor(0,2); //set cursor to line 2, posiion 0
    lcd.print("OT:x.");
    lcd.print(xo);
    lcd.print(" y.");
    lcd.print(yo);
  */
}
//==========================================================================
//     LOOP     LOOP     LOOP     LOOP     LOOP     LOOP     LOOP     LOOP
//==========================================================================
void loop() {
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
  if (toggle) {   // display once and toggle
    lcd.clear();
    lcd.print(data);
    toggle = false;
  }
  else{
    
  }
  
}
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//     receiveEvent    receiveEvent     receiveEvent     receiveEvent
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  Read from I2C bus
void receiveEvent(int bytes) {
  data = "";
  while ( Wire.available()) {
    data += (char)Wire.read();
    Serial.println(data);
  }
  toggle = true;  //set toggle to display once
}
