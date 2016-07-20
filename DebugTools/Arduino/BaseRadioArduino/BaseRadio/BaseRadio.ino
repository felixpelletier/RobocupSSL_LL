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

const byte STARTBYTE = 0x7E;
const byte STOPBYTE = 0x7F;
const byte ESCAPEBYTE = 0x7D;
const byte SPEEDCOMMAND_ID = 0x1;

const bool DEBUG = false;

char inputString[200];
int inputIndex = 0;
bool stringComplete = false;
bool isNextByteAnEscape = false;

const int PAYLOAD_SIZE = 15;

const int ledpin = 4;
bool ledstate = true;

struct payload_data{
  byte robot_id;
  byte command_type;
  float x;
  float y;
  float theta;
  //byte robot_id;
  byte padding[1];
};

union payload_t {
  byte bytes[PAYLOAD_SIZE];
  struct payload_data data;
} payload;

void setup() {
  Serial.begin(115200);
  pinMode(ledpin, OUTPUT);

  printf_begin();

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

void loop() {
  if (stringComplete) {
    if (DEBUG){
      Serial.println(inputString);
      for(int i = 0; i<PAYLOAD_SIZE; i++){
        payload.bytes[i] = inputString[i];
      }
      Serial.print(" Robot ID: ");
      Serial.print(payload.data.robot_id);
      Serial.print(" X: ");
      Serial.print(payload.data.x);
      Serial.print(" Y: ");
      Serial.print(payload.data.y);
      Serial.print(" Theta: ");
      Serial.print(payload.data.theta);
      Serial.println("");
      digitalWrite(ledpin, HIGH);
    }
    else{
      sendPacket(inputString);
    }
    toggleLED();
    inputIndex = 0;
    stringComplete = false;
    
  }
}

void toggleLED(){

  if (ledstate){
    digitalWrite(ledpin, HIGH);
  }
  else{
    digitalWrite(ledpin, LOW);
  }

  ledstate = !ledstate;
  
}

void sendPacket(const char* raw_packet){
    unsigned char transformed_packet[PAYLOAD_SIZE];
    transformed_packet[0] = raw_packet[1];
    
    /*transformed_packet[1] = raw_packet[5];
    transformed_packet[2] = raw_packet[4];
    transformed_packet[3] = raw_packet[3];
    transformed_packet[4] = raw_packet[2];

    transformed_packet[5] = raw_packet[9];
    transformed_packet[6] = raw_packet[8];
    transformed_packet[7] = raw_packet[7];
    transformed_packet[8] = raw_packet[6];

    transformed_packet[9] = raw_packet[13];
    transformed_packet[10] = raw_packet[12];
    transformed_packet[11] = raw_packet[11];
    transformed_packet[12] = raw_packet[10];*/

    transformed_packet[1] = raw_packet[2];
    transformed_packet[2] = raw_packet[3];
    transformed_packet[3] = raw_packet[4];
    transformed_packet[4] = raw_packet[5];

    transformed_packet[5] = raw_packet[6];
    transformed_packet[6] = raw_packet[7];
    transformed_packet[7] = raw_packet[8];
    transformed_packet[8] = raw_packet[9];

    transformed_packet[9] = raw_packet[10];
    transformed_packet[10] = raw_packet[11];
    transformed_packet[11] = raw_packet[12];
    transformed_packet[12] = raw_packet[13];
    
    transformed_packet[13] = raw_packet[0];
    
    radio.write( transformed_packet, PAYLOAD_SIZE );
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    if (inChar == STARTBYTE && !isNextByteAnEscape){
      inputIndex = 0;
    }
    else if(inChar == STOPBYTE && !isNextByteAnEscape){
      stringComplete = true;
    }
    else if(inChar == ESCAPEBYTE && !isNextByteAnEscape){
      isNextByteAnEscape = true;
    }
    else{
      inputString[inputIndex++] = inChar;
      isNextByteAnEscape = false;
    }

  }
}
