/*
    Robocup Protocol V1.0

    | Robot's Id | Command | Data |
    Velocity command :
    |id|1|Vx|Vy|Vtheta|
    char|char|float|float|float

    -----
    PID command :
    |id|2|P|I|D|
    char|char|float|float|float

*/


#ifndef PACKER_H
#define PACKER_H

#include <queue>
#include <iostream>
#include "Serial.h"

using namespace std;

typedef union dataConverter {
    float floatValue;
    int intValue;
    unsigned char charValues[4];
}dataConverter;


class Packer{
public:
    Packer(Serial * pSerial);
    void createSpeedCommand(float iX, float iY, float iTheta, unsigned char iId);
    void createSetPidCommand(float iP, float iI, float iD, unsigned char iId); //It could be the same function but for now it read better
    static unsigned char STARTBYTE;
    static unsigned char STOPBYTE;
    static unsigned char ESCAPEBYTE;

    void sendPacket();
    void printPacket();

private:
    void pppProtocol();
    void insertFloatInPacket(float iData);
    void insertIntInPacket(int iData);
    void insertStartByteInPacket();
    void insertStopByteInPacket();

    dataConverter mDataConverter;
    Serial * mSerial;
    queue<unsigned char> mPacket;

};



#endif

