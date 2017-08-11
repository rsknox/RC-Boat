/*
Arduino Hall Effect Sensor Project
by Arvind Sanjeev
Please check out  http://diyhacking.com for the tutorial of this project.
DIY Hacking
*/


 volatile byte half_revolutions;
 byte half_revolutionsp;
 unsigned int rpm;
 unsigned long timeold;
 long detectn = 0;
 long detectp = 0;
 void setup()
 {
   digitalWrite(2,1);   
   Serial.begin(9600);
   attachInterrupt(0, magnet_detect, CHANGE);//Initialize the intterrupt pin (Arduino digital pin 2)
   half_revolutions = 0;
   half_revolutionsp = 0;
   rpm = 0;
   timeold = 0;
 }
 void loop()//Measure RPM
 {
  
  if(half_revolutions != half_revolutionsp)
  {
    detectn = detectn + 1;
    Serial.print("Detection count = ");
    Serial.println(detectn);
    half_revolutionsp = half_revolutions;
  }
   if (half_revolutions >= 20) { 
     rpm = 30*1000/(millis() - timeold)*half_revolutions;
     timeold = millis();
     half_revolutions = 0;
     Serial.println(rpm,DEC);
   }
 }
 void magnet_detect()//This function is called whenever a magnet/interrupt is detected by the arduino
 {
   half_revolutions++;
 //  Serial.print("detect; half_resolutions =  ");
 //  Serial.println(half_revolutions);
 }
