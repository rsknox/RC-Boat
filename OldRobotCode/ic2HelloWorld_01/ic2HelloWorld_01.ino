//Simple i2c LCD Hello World
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F,2,1,0,4,5,6,7,3,POSITIVE);
int xi = 750;
int yi = 1004;
int xo = 749;
int yo = 749;
void setup() {
  // put your setup code here, to run once:
lcd.begin(16,2);
lcd.clear();
lcd.print("System Ready");
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

void loop() {
  // put your main code here, to run repeatedly:

}
