//swarm controller Nano 1
/*
 Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * Example: Network topology, and pinging across a tree/mesh network
 *
 * Using this sketch, each node will send a ping to every other node in the network every few seconds. 
 * The RF24Network library will route the message across the mesh to the correct node.
 *
 * This sketch is greatly complicated by the fact that at startup time, each
 * node (including the base) has no clue what nodes are alive.  So,
 * each node builds an array of nodes it has heard about.  The base
 * periodically sends out its whole known list of nodes to everyone.
 *
 * To see the underlying frames being relayed, compile RF24Network with
 * #define SERIAL_DEBUG.
 *
 * Update: The logical node address of each node is set below, and are grouped in twos for demonstration.
 * Number 0 is the master node. Numbers 1-2 represent the 2nd layer in the tree (02,05).
 * Number 3 (012) is the first child of number 1 (02). Number 4 (015) is the first child of number 2.
 * Below that are children 5 (022) and 6 (025), and so on as shown below 
 * The tree below represents the possible network topology with the addresses defined lower down
 *
 *     Addresses/Topology                            Node Numbers  (To simplify address assignment in this demonstration)
 *             00                  - Master Node         ( 0 )
 *           02  05                - 1st Level children ( 1,2 )
 *    32 22 12    15 25 35 45    - 2nd Level children (7,5,3-4,6,8)
 *
 * eg:
 * For node 4 (Address 015) to contact node 1 (address 02), it will send through node 2 (address 05) which relays the payload
 * through the master (00), which sends it through to node 1 (02). This seems complicated, however, node 4 (015) can be a very
 * long way away from node 1 (02), with node 2 (05) bridging the gap between it and the master node.
 *
 * To use the sketch, upload it to two or more units and set the NODE_ADDRESS below. If configuring only a few
 * units, set the addresses to 0,1,3,5... to configure all nodes as children to each other. If using many nodes,
 * it is easiest just to increment the NODE_ADDRESS by 1 as the sketch is uploaded to each device.
 */

#include <avr/pgmspace.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include "printf.h"
#include <math.h>
/***********************************************************************
************* Set the Node Address *************************************
/***********************************************************************/

// These are the Octal addresses that will be assigned
const uint16_t node_address_set[6] = { 00, 01, 011, 021, 031, 041};
 
// 0 = Master
// 1-2 (02,05)   = Children of Master(00)
// 3,5 (012,022) = Children of (02)
// 4,6 (015,025) = Children of (05)
// 7   (032)     = Child of (02)
// 8,9 (035,045) = Children of (05)

uint8_t NODE_ADDRESS = 0;  // Use numbers 0 through to select an address from the array

/***********************************************************************/
/***********************************************************************/


RF24 radio(7,8);                              // CE & CS pins to use (Using 7,8 on Uno,Nano)
RF24Network network(radio); 

uint16_t this_node;                           // Our node address

const unsigned long interval = 20; // ms       // Delay manager to send pings regularly.
unsigned long last_time_sent;
#define JOYSTICK_X   A0  // The Joystick potentiometers connected to Arduino Analog inputs
#define JOYSTICK_Y   A1
#define JOYSTICK_SW  A2  // The Joystick push-down switch, will be used as a Digital input

const short max_active_nodes = 10;            // Array of nodes we are aware of
uint16_t active_nodes[max_active_nodes];
short num_active_nodes = 0;
short next_ping_node_index = 0;


bool send_T(uint16_t to);                      // Prototypes for functions to send & handle messages
bool send_N(uint16_t to);
bool send_J(uint16_t to);
void handle_T(RF24NetworkHeader& header);
void handle_N(RF24NetworkHeader& header);
void handle_J(RF24NetworkHeader& header);
void add_node(uint16_t node);
/**
  Create a data structure for transmitting and receiving data
  This allows many variables to be easily sent and received in a single transmission
  See http://www.cplusplus.com/doc/tutorial/structures/
*/
struct dataStruct {
  unsigned long _micros;  // to save response times
  int Xposition;          // The Joystick position values
  int Yposition;
  bool switchOn;          // The Joystick push-down switch
} myData;                 // This can be accessed in the form:  myData.Xposition  etc.

struct msg_J {          // seting up mesage to send cordinents to the flight leader
  int x;
  int y;
  bool sw;
};

void setup(){
  //Serial.begin(9600);  // MUST reset the Serial Monitor to 115200 (lower right of window )
  // NOTE: The "F" in the print statements means "unchangable data; save in Flash Memory to conserve SRAM"
  //Serial.println(F("YourDuino.com Example: Send joystick data by nRF24L01 radio to another Arduino"));
  //printf_begin(); // Needed for "printDetails" Takes up some memory
  pinMode(JOYSTICK_SW, INPUT_PULLUP);  // Pin A2 will be used as a digital input

  Serial.begin(9600);
  printf_begin();
  printf_P(PSTR("\n\rRF24Network/examples/meshping/\n\r"));

  this_node = node_address_set[NODE_ADDRESS];            // Which node are we?
  
  SPI.begin();                                           // Bring up the RF network
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  network.begin(/*channel*/ 110, /*node address*/ this_node );

}

void loop(){
        /*********************( Read the Joystick positions )*************************/
    myData.Xposition = analogRead(JOYSTICK_X);
    myData.Yposition = analogRead(JOYSTICK_Y);
    myData.switchOn  = !digitalRead(JOYSTICK_SW);  // Invert the pulldown switch

  myData._micros = micros();  // Send back for timing

  network.update();                                      // Pump the network regularly

   while ( network.available() )  {                      // Is there anything ready for us?
     
    RF24NetworkHeader header;                            // If so, take a look at it
    network.peek(header);
    //Serial.print(" header type ");
    //Serial.print(header.type);
    
      switch (header.type){                              // Dispatch the message to the correct handler.
        case 'T': handle_T(header); break;
        case 'N': handle_N(header); break;
        default:  printf_P(PSTR("*** WARNING *** Unknown message type %c\n\r"),header.type);
                  network.read(header,0,0);
                  break;
      };
    }

  
  unsigned long now = millis();                         // Send a ping to the next node every 'interval' ms
  if ( now - last_time_sent >= interval ){
    last_time_sent = now;


    uint16_t to = 00;                                   // Who should we send to? By default, send to base
    
    
    if ( num_active_nodes ){                            // Or if we have active nodes,
        to = active_nodes[next_ping_node_index++];      // Send to the next active node
        if ( next_ping_node_index > num_active_nodes ){ // Have we rolled over?
	    next_ping_node_index = 0;                   // Next time start at the beginning
	    to = 00;                                    // This time, send to node 00.
        }
    }

    bool ok;

    
    if ( this_node > 00 || to == 00 ){                    // Normal nodes send a 'T' ping
       //Serial.println ("send_t");
        ok = send_T(to);   
    }else{                           // Base node sends the current active nodes out
      //Serial.println("send_n");
        ok = send_N(to);
 //   }
    
    if (ok){                                              // Notify us of the result
        printf_P(PSTR("%lu: APP Send ok\n\r"),millis());
    }else{
        printf_P(PSTR("%lu: APP Send failed\n\r"),millis());
        last_time_sent -= 100;                            // Try sending at a different time next time
    }
  ok = send_J(to);
  if (ok){
    printf_P(PSTR("%lu: APP Send_J ok\n\r"),millis());
  }else{
    printf_P(PSTR("%lu: APP Send_J failed\n\r"),millis());
    last_time_sent -=100;
  }
  }


//  delay(50);                          // Delay to allow completion of any serial printing
//  if(!network.available()){
//      network.sleepNode(2,0);         // Sleep this node for 2 seconds or a payload is received (interrupt 0 triggered), whichever comes first
//  }
}
}
/**
 * Send a 'T' message, the current time
 */
bool send_T(uint16_t to)
{
  RF24NetworkHeader header(/*to node*/ to, /*type*/ 'T' /*Time*/);
  
  // The 'T' message that we send is just a ulong, containing the time
  unsigned long message = millis();
  printf_P(PSTR("---------------------------------\n\r"));
  printf_P(PSTR("%lu: APP Sending %lu to 0%o...\n\r"),millis(),message,to);
  return network.write(header,&message,sizeof(unsigned long));
}

/**
 * Send an 'N' message, the active node list
 */
bool send_N(uint16_t to)
{
  RF24NetworkHeader header(/*to node*/ to, /*type*/ 'N' /*Time*/);
  
  printf_P(PSTR("---------------------------------\n\r"));
  printf_P(PSTR("%lu: APP Sending active nodes to 0%o...\n\r"),millis(),to);
  return network.write(header,active_nodes,sizeof(active_nodes));
}

/**
 * Send an 'J' message, the active node list
 */
bool send_J(uint16_t to)
{
  RF24NetworkHeader header(/*to node*/ to, /*type*/ 'J' /*Time*/);
  Serial.println("entering send J");
  printf_P(PSTR("---------------------------------\n\r"));
  printf_P(PSTR("%lu: APP Sending active nodes to 0%o...\n\r"),millis(),to);
  int x = myData.Xposition;
  int y = myData.Yposition;
  bool sw = myData.switchOn;
  Serial.print("Raw joystick coords: x= ");
  Serial.print(x);
  Serial.print("  y= ");
  Serial.println(y);
 // caculation to restrict y to forward motion only an x to + or - 45 degrees 
 if (y < 558) {             //y = 558 is the null posion on our joy stick
  y = 558;              // if the y is less than 558 we restrict to 558 so we dont go backwards
 }
 if (x == 550){
  x = 551;      //making sure x dont = 550 so theres no / 0
 }
//  float tan = ((y - 558) / (x - 550));
//  Serial.print("Tan of joystick coords: ");
//  Serial.print(tan);
//  double angle = atan(tan);
//  Serial.print("   Angle of joystick coords: ");
//  Serial.println(angle);
//  if(abs(tan) < 1.0){  //if tan is less than 1.0, that means the turn angle is greater than 45 degrees
////  angle = 1.57 - angle;      //we are subtracting the angle pi over 2 to determin the angle ffrom strait a head  
////  if (angle > .785) {      // if the angle > than pi/4 make x = to y to put it at 45 dregrees
//  if (tan < 0.00){   // if it's negative, make negative x equal to y to ensure 45 degrees
//    x = -y;
//  }
// else{
//  x = y;
//  }
//  }  // if it falls through, let x,y equal the input joystick coords
//  Serial.print(" angle: ");
//  Serial.println(angle);
  msg_J msg = {x, y, sw};
  Serial.print("outgoing msg : x: ");
  Serial.print(x);
  Serial.print(" y ");
  Serial.print(y);
  Serial.print(" sw ");
  Serial.print(sw);
  
  return network.write(header, &msg, sizeof (msg));
}
/**
 * Handle a 'T' message
 * Add the node to the list of active nodes
 */
void handle_T(RF24NetworkHeader& header){

  unsigned long message;                                                                      // The 'T' message is just a ulong, containing the time
  network.read(header,&message,sizeof(unsigned long));
  printf_P(PSTR("%lu: APP Received %lu from 0%o\n\r"),millis(),message,header.from_node);


  if ( header.from_node != this_node || header.from_node > 00 )                                // If this message is from ourselves or the base, don't bother adding it to the active nodes.
    add_node(header.from_node);
}

/**
 * Handle an 'N' message, the active node list
 */
void handle_N(RF24NetworkHeader& header)
{
  static uint16_t incoming_nodes[max_active_nodes];

  network.read(header,&incoming_nodes,sizeof(incoming_nodes));
  printf_P(PSTR("%lu: APP Received nodes from 0%o\n\r"),millis(),header.from_node);

  int i = 0;
  while ( i < max_active_nodes && incoming_nodes[i] > 00 )
    add_node(incoming_nodes[i++]);
}

/**
 * Add a particular node to the current list of active nodes
 */
void add_node(uint16_t node){
  
  short i = num_active_nodes;                                    // Do we already know about this node?
  while (i--)  {
    if ( active_nodes[i] == node )
        break;
  }
  
  if ( i == -1 && num_active_nodes < max_active_nodes ){         // If not, add it to the table
      active_nodes[num_active_nodes++] = node; 
      printf_P(PSTR("%lu: APP Added 0%o to list of active nodes.\n\r"),millis(),node);
  }
}


