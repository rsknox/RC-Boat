//this is going to be our dead recoing and wheel shaft encoder program
// based on 
/*
Arduino Hall Effect Sensor Project
by Arvind Sanjeev
Please check out  http://diyhacking.com for the tutorial of this project.
DIY Hacking
*/


 volatile byte half_revolutionsl;
 byte half_revolutionspl;
 volatile byte half_revolutionsr;
 byte half_revolutionspr;
 unsigned int rpm;
 unsigned long timeold;
 long detectl = 0;
 long detectr = 0;
 void setup()
 {
   digitalWrite(2,1);  // define left wheel shaft encoder pin = D2
   digitalWrite(3,1);  // define right wheel shaft encoder pin = D3
   Serial.begin(9600);
   attachInterrupt(0, magnet_detectl, CHANGE);//Initialize the intterrupt pin (Arduino digital pin 2)
   attachInterrupt(1, magnet_detectr, CHANGE);//Initialize the intterrupt pin (Arduino digital pin 3)
   half_revolutionsl = 0;
   half_revolutionspl = 0;
     half_revolutionsr = 0;
   half_revolutionspr = 0;
   rpm = 0;
   timeold = 0;
 }
 void loop()//Measure RPM
 {
  
  if(half_revolutionsl != half_revolutionspl)
  {
    detectl = detectl + 1;
    Serial.print("Detection left count = ");
    Serial.println(detectl);
    half_revolutionspl = half_revolutionsl;
  }

if(half_revolutionsr != half_revolutionspr)
  {
    detectr = detectr + 1;
    Serial.print("Detection right count = ");
    Serial.println(detectr);
    half_revolutionspr = half_revolutionsr;
  }
  
   if (half_revolutionsl >= 20) { 
     rpm = 30*1000/(millis() - timeold)*half_revolutionsl;
     timeold = millis();
     half_revolutionsl = 0;
     Serial.println(rpm,DEC);
   }
 }
 void magnet_detectl()//This function is called whenever a magnet/interrupt is detected by the arduino
 {
   half_revolutionsl++;
 //  Serial.print("detect; half_resolutions =  ");
 //  Serial.println(half_revolutions);
 }
 void magnet_detectr()//This function is called whenever a magnet/interrupt is detected by the arduino
 {
   half_revolutionsr++;
 //  Serial.print("detect; half_resolutions =  ");
 //  Serial.println(half_revolutions);
 }
