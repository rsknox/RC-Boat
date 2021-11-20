// Motor controller for 21" cabin cruiser RC boat
// 12 Nov 2021

int In1 = 7; //pin for rotation direct'n
int In2 = 8; //pin for rotation direct'n
int En1 = 5; //PWM pin
int SPEED = 255; //rotation speed
  
//  const byte controllerFA = 9; //PWM FORWARD PIN for OSMC Controller A
//  const byte controllerRA = 6; //PWM REVERSE PIN for OSMC Controller A
//  int rvel; // reverse velocity (0-255)
//  int fvel; // forward velocity



void setup() {
  Serial.begin(9600);
  Serial.println( __FILE__ );
  Serial.println( __DATE__ );
  Serial.println( __TIME__ );

  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(En1,OUTPUT);

  digitalWrite(In1,HIGH);
  digitalWrite(In2,LOW);

  analogWrite(En1,SPEED);

//  pinMode(controllerFA, OUTPUT);
//  pinMode(controllerRA, OUTPUT);
//  pinMode(9, OUTPUT);
//  pinMode(6, OUTPUT);
//
//  rvel = 64;
//  fvel = 0;

}

void loop() {
  
    //analogWrite(En1,SPEED);
  
  //Serial.println("loop top");
  //analogWrite(controllerRA, rvel); //apply the velocity PWM values to the motor
  //rvel = 255;
//  analogWrite(controllerRA, rvel); //apply the velocity PWM values to the motor
//  delay(2000);
  //analogWrite(controllerRA,0);
//  analogWrite(controllerRA,0);
//  delay(2000);
  
  //analogWrite(controllerFA, fvel);
}
