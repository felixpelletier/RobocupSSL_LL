/*
  // March 2014 - TMRh20 - Updated along with High Speed RF24 Library fork
  // Parts derived from examples by J. Coliz <maniacbug@ymail.com>
*/
/**
 * Example for efficient call-response using ack-payloads 
 *
 * This example continues to make use of all the normal functionality of the radios including
 * the auto-ack and auto-retry features, but allows ack-payloads to be written optionally as well.
 * This allows very fast call-response communication, with the responding radio never having to 
 * switch out of Primary Receiver mode to send back a payload, but having the option to if wanting
 * to initiate communication instead of respond to a commmunication.
 */
 


#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 
RF24 radio(7,8);

// Topology
const uint64_t pipes[2] = { 0xE7E7E7E7E7LL, 0xE7E7E7E7E8LL };              // Radio pipe addresses for the 2 nodes to communicate.

// Role management: Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.  

typedef enum { role_ping_out = 1, role_pong_back } role_e;                 // The various roles supported by this sketch
const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};  // The debug-friendly names of those roles
role_e role = role_ping_out;                                              // The role of the current running sketch

// A single byte to keep track of the data being sent back and forth
byte counter = 1;

unsigned long last_payload_time;
int ledPin = 4;

#define PAYLOAD_SIZE 15

struct payload_data{
  //byte robot_id;
  byte command_type;
  float x;
  float y;
  float theta;
  byte padding2[2];
};

union payload {
  byte bytes[PAYLOAD_SIZE];
  struct payload_data data;
} payload;


void setup(){

  Serial.begin(115200);
  printf_begin();
  Serial.print(F("\n\rRF24/examples/pingpair_ack/\n\rROLE: "));
  Serial.println(role_friendly_name[role]);
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));

  pinMode(ledPin, OUTPUT);

  // Setup and configure rf radio

  radio.begin();
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  //radio.enableAckPayload();               // Allow optional ack payloads
  radio.setRetries(0,15);                 // Smallest time between retries, max no. of retries
  radio.setPayloadSize(PAYLOAD_SIZE);                // Here we are sending 1-byte payloads to test the call-response speed
  radio.setChannel(2);
  radio.setCRCLength(RF24_CRC_8);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openWritingPipe(pipes[0]);        // Both radios listen on the same pipes by default, and switch when writing
  radio.openReadingPipe(1,pipes[1]);
  radio.stopListening();                 // Start listening
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging
}

void loop(void) {

  if (role == role_ping_out){
    
    radio.stopListening();                                  // First, stop listening so we can talk.

    float speed = 2.0;
    float side = 2.0 * 0.707f;

    payload.data.command_type = 1;
    payload.data.theta = 0;

    
    payload.data.x = side;
    payload.data.y = side;
    radio.write( &payload.bytes, 15 );
    
    delay(500);

    payload.data.x = 0;
    payload.data.y = 0;
    radio.write( &payload.bytes, 15 );

    delay(2000);

    payload.data.x = side;
    payload.data.y = -side;
    radio.write( &payload.bytes, 15 );
    
    delay(500);

    payload.data.x = 0;
    payload.data.y = 0;
    radio.write( &payload.bytes, 15 );
    
    delay(2000);
  }
  
  // Pong back role.  Receive each packet, dump it out, and send it back

  if ( role == role_pong_back ) {
    byte pipeNo;
    while( radio.available(&pipeNo)){
      radio.read( &payload.bytes, PAYLOAD_SIZE );
      Serial.print("Payload: ");
      /*for (int i=0; i < PAYLOAD_SIZE; i++)
      {
        Serial.print(payload.bytes[i]);
        Serial.print(" ");
      }*/
      //Serial.print("Robot ID: ");
      //Serial.print(payload.data.robot_id);
      Serial.print("X: ");
      Serial.print(payload.data.x);
      Serial.print(" Y: ");
      Serial.print(payload.data.y);
      Serial.print(" Theta: ");
      Serial.print(payload.data.theta);
      Serial.println("");

      last_payload_time = millis();
      digitalWrite(ledPin, HIGH);
      //radio.writeAckPayload(pipeNo,&gotByte, 1 );    
   }
 }

  if ( millis()-last_payload_time > 1000 ){
      digitalWrite(ledPin, LOW);
  }
  // Change roles

  if ( Serial.available() )
  {
    char c = toupper(Serial.read());
    if ( c == 'T' && role == role_pong_back )
    {
      Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));

      role = role_ping_out;                  // Become the primary transmitter (ping out)
      radio.openWritingPipe(pipes[0]);
      radio.openReadingPipe(1,pipes[1]);
    }
    else if ( c == 'R' && role == role_ping_out )
    {
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
      
       role = role_pong_back;                // Become the primary receiver (pong back)
       radio.openWritingPipe(pipes[1]);
       radio.openReadingPipe(1,pipes[0]);
       radio.startListening();
    }
    radio.printDetails();
  }
}
