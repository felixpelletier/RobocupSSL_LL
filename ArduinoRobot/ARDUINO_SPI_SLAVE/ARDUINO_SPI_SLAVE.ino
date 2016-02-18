
// Written by Philippe Babin 
// Based on a script by Nick Gammon
// Created : 20 November 2013


//#include <SPI.h> // does not support spi slave mode
#include <Wire.h>
#include <Adafruit_INA219.h>


//stuff for the spi slave communication
boolean SSlast = LOW;         // SS last flag.
const byte led = 9;           // Slave LED digital I/O pin.
boolean ledState = HIGH;      // LED state flag.
//const byte cmdBtn = 1;        // SPI cmdBtn master command code.
//const byte cmdLEDState = 2; 

// added commands
const byte cmdActivateDribbler = 0x22u;
const byte cmdDeactivateDribbler = 0x33u;

//function to Initialize SPI slave.
void SlaveInit(void) {
  // Initialize SPI pins.
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, INPUT);
  pinMode(SS, INPUT);
  // Enable SPI as slave.
  SPCR = (1 << SPE);
}

// function for SPI Transfer. it sends a value then reads another
byte SPItransfer(byte value) {
  SPDR = value;
  while(!(SPSR & (1<<SPIF)));
  delay(10);
  return SPDR;
}





// Read command
const byte      ARDUINO_BAT_MONITOR_1       =    0x01;
const byte       ARDUINO_BAT_MONITOR_2       =    0x02;
const byte       ARDUINO_RECALL              =    0x10;
// Writing command
const byte       ARDUINO_GPIO_2              =	   0xB2;
const byte       ARDUINO_GPIO_3              =	   0xB3;
const byte       ARDUINO_GPIO_4              =	   0xB4;
const byte       ARDUINO_GPIO_5              =	   0xB5;
const byte       ARDUINO_GPIO_6              =	   0xB6;
const byte       ARDUINO_GPIO_7              =	   0xB7;
const byte       ARDUINO_GPIO_8              =	   0xB8;
const byte       ARDUINO_GPIO_9              =	   0xB9;

const int       ARDUINO_LOW                 =	   0;
const int       ARDUINO_HIGH                =	   1;
const byte       ARDUINO_RECALL_MESSAGE      =	   0xEE;

const byte       ARDUINO_WRITE_FLAG          =    0x80;


const int greenPin = 8; 
const int redPin = 4; 
const int bluePin = 9; 
const int initialize_delay = 2000; 

byte reg;
byte value;
float batV7;
float batV16;
float batP7;
float batP16;

float MIN = 0;
float MAX = 4.0;

enum wait_state_t {
  WAITING_COMMAND,
  WAITING_VALUE
};
enum spi_state_t {
  READ_COMMAND,
  WRITE_COMMAND
};

spi_state_t  stateSPICommand = READ_COMMAND;
wait_state_t  stateSPIWrite = WAITING_COMMAND;
wait_state_t  stateSPIRead = WAITING_COMMAND;
byte  command = 0;


boolean gotAReg = false;
boolean gotAValue = false;

#define BUFFER_SIZE 256
byte rev_buffer[BUFFER_SIZE];

unsigned int index_write = 0;
unsigned int index_read = 0;



// Arduino Promini communication SPI:
/// 10 (SS), 11 (MOSI), 12 (MISO), 13 (SCK)

// INA219 I2C address:
/***
 * A1  |  A0  |  adresse | Hex
 * GND    GND    1000000   40
 * GND    Vs     1000001   41
 * GND    SDA    1000010   42
 * GND    SCL    1000011   43
 * Vs     GND    1000100   44
 * Vs     Vs     1000101   45
 */
Adafruit_INA219 monV7(0x45);
Adafruit_INA219 monV16(0x40);

void setup (void)
{
  Serial.begin (115200);   // Serial debugging

  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);


 Serial.println("Séquence de démarrage");
  //initsequence(); //Sequence d'initialisation
  //delay(initialize_delay);
 
  // GPIO
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW); // Dribbler
  digitalWrite(6, HIGH); // Charger Capacitor
  digitalWrite(7, HIGH);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);

  //analogWrite(5, 128); // 50% of pwm for dribbler
  Serial.println("Relais déclenché");

  // now turn on interrupts for Master request, not needed in this version
  //SPI.attachInterrupt();

  // We start the battery monitoring
  monV7.begin();
  monV16.begin();

  // Initialize SPI Slave.
  SlaveInit();
  Serial.println("Slave Initialized");

}  

void updateGPIO(int n, int state){
  if( state == ARDUINO_LOW)
    digitalWrite(n, LOW);
  else if( state == ARDUINO_HIGH)
    digitalWrite(n, HIGH);
  else
    Serial.println("Error");
}


void loop (void){
  //digitalWrite(6, LOW); // Charger Capacitor
  //delay(1000);
  //digitalWrite(6, HIGH); // Charger Capacitor
  //delay(1000);



  
  //communication spi
  
  // Slave Enabled?
  if (!digitalRead(SS)) {
    // Yes, first time?
    if (SSlast != LOW) {
      // Yes, take MISO pin.
      pinMode(MISO, OUTPUT);
      //Serial.println("***Slave Enabled.");
      // Write 0x99 slave response code and receive master command code
      byte rx = SPItransfer(0x99);
      //Serial.println("Initial -1 slave response code sent");
      //Serial.println("rx:" + String(rx) + ".");
      Serial.println("rx =");
      Serial.println(rx);
      // COMMANDES
      // cmdActivateDribbler?
      if (rx == cmdActivateDribbler) {
        // Turn Dribbler On
        Serial.println("exterminate!");
        updateGPIO(5, ARDUINO_HIGH);
      }
      // cmdDeactivateDribbler?
      else if (rx == cmdDeactivateDribbler) {
        // Turn Dribbler Off
        Serial.println("save humans plz kthx");
        updateGPIO(5, ARDUINO_LOW);
      }
      
      // INFORMATION REQUESTS
      // ARDUINO_RECALL?
      else if (rx == ARDUINO_RECALL) {
        // Acknowledge
        
        byte rx = SPItransfer(ARDUINO_RECALL_MESSAGE);
        
        Serial.println("recall sent ");  
      }
      // ARDUINO_BAT_MONITOR_1 ?
      else if (rx == ARDUINO_BAT_MONITOR_1) {
        // Acknowledge cmdLEDState.
        byte rx = SPItransfer(ARDUINO_BAT_MONITOR_1 );
        //Serial.println("cmdLEDState Acknowledged.");
        //Serial.println("rx:" + String(rx) + ".");
        rx = SPItransfer(batV7);
        //Serial.println("ledState:" + String(ledState) + " Sent.");
        //Serial.println("rx:" + String(rx) + ".");        
      }
      // ARDUINO_BAT_MONITOR_2 ?
      else if (rx == ARDUINO_BAT_MONITOR_2) {
        // Acknowledge cmdLEDState.
        byte rx = SPItransfer(ARDUINO_BAT_MONITOR_2 );
        //Serial.println("cmdLEDState Acknowledged.");
        //Serial.println("rx:" + String(rx) + ".");
        rx = SPItransfer(batV16);
        //Serial.println("ledState:" + String(ledState) + " Sent.");
        //Serial.println("rx:" + String(rx) + ".");        
      }
      
      else {
        // Unrecognized command.
        byte rx = SPItransfer(rx);
        //Serial.println("Unrecognized Command -1 slave response code sent.");
        //Serial.println("rx:" + String(rx) + ".");
      }
      // Update SSlast.
      SSlast = LOW;
    }
  }
  else {
    // No, first time?
    if (SSlast != HIGH) {
      // Yes, release MISO pin.
      pinMode(MISO, INPUT);
      //Serial.println("Slave Disabled.");
      // Update SSlast.
      SSlast = HIGH;
    }
  }

// end of spi communication here







  // - 32768 a 32767
  // float current_mA = 0;
  //float loadvoltage = 0;
  //current_mA = ina219.getCurrent_mA();
  //batV7 = map(monV7.getBusVoltage_V() + (monV7.getShuntVoltage_mV() / 1000), MIN, MAX, 0,255);

  batV7 = monV7.getBusVoltage_V() + (monV7.getShuntVoltage_mV() / 1000);
  batV16 = monV16.getBusVoltage_V() + (monV16.getShuntVoltage_mV() / 1000);
  batP7 = batV7 * monV7.getCurrent_mA();
  batP16 = batV16 * monV16.getCurrent_mA();
  //Serial.print("I:");Serial.println(monV16.getCurrent_mA());
  
  //Serial.print(batV7);
  //Serial.print(" ");
  //Serial.println(batV16);
  if(batV7 > 6 && batV16 > 12 && batV16 < 30)//Vérifie que la batterie a une tension suppérieure à 12 V
    setColor(0, 170, 0); // Green
  else if(batV16 <= 12){  //Vérifie que la batterie a une tension suppérieure à 12 V
    setColor(170, 0, 0); // Red
    digitalWrite(7, LOW);
  }
  else
    setColor(170, 0, 0); // Red

}  // end of loop


//sequence d'initialisation (just for fun)
void initsequence()
{
  setColor(0, 0, 170); // blue
  delay(initialize_delay);

  setColor(0, 170, 0); // Green
  delay(initialize_delay);

  setColor(170, 0, 0); // Red
  delay(initialize_delay);    
}//----FIN--- de l'inialisation

//Fonction afin de faire allumer la led RGB
void setColor(int red, int green, int blue) 
{
  analogWrite(redPin, red); 
  analogWrite(greenPin, green); 
  analogWrite(bluePin, blue); 
}


