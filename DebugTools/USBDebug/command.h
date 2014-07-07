#ifndef COMMAND_H
#define COMMAND_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "Serial.h"
#include "packer.h"
#ifdef __WINDOWS__
  #include <windows.h>
#endif
#ifdef __linux__
    #include <unistd.h>
#endif
using namespace std;

void printHelp();
Serial* connectToPort();
void printSpeedPacket(float iX, float iY, float iTheta, int iId);
void sendPacket(float iHz, int iQuantity,Serial *iSerial);

#endif
