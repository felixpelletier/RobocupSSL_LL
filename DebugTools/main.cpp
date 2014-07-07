#include <iostream>
#include "USBDebug/Serial.h"
#include "USBDebug/command.h"

using namespace std;

int main()
{
    cout << "---C2000 USB-Debug Tool V_0.1---" << endl;
    cout << "Enter help for command list" << endl;

    string lCommand;
    Serial *lSerial = NULL;

    while(1){

        cout << ">>>";
        cin >> lCommand;

        if(lCommand == "help" || lCommand == "h"){
            printHelp();
        }
        else if(lCommand == "connect" || lCommand == "cn"){
            lSerial = connectToPort();
        }
        else if(lCommand == "sendbuffer" || lCommand == "sb"){
            unsigned int bufferSize = 0;
            cout << "send buffer size :" << endl;
            cin >> bufferSize;
            char buffer[bufferSize];
            unsigned char inc = 0;
            for(int i = 0; i < bufferSize; ++i){
                buffer[i] = inc;
                ++inc;
            }
            if(lSerial->IsConnected()){
                lSerial->WriteData(buffer,bufferSize);
            }

        }
        else if(lCommand == "printPacket" || lCommand == "pp"){
            unsigned char lId = 1;
            float lX;
            float lY;
            float lTheta;
            cout << endl << "X Speed :";
            cin >> lX;
            cout << endl  << "Y Speed :";
            cin >> lY;
            cout << endl  << "Theta Speed :";
            cin >> lTheta;
            cout << endl;
            printSpeedPacket(lX, lY, lTheta, lId);
        }
        else if(lCommand == "sendPacket" || lCommand == "sp"){
            float hz = 0;
            int quantity = 1;
            cout << endl << "Number of packet :";
            cin >> quantity;
            cout << endl << "Frequency (hz) :";
            cin >> hz;
            sendPacket(hz,quantity,lSerial);
        }
        else if(lCommand == "read"){
            char buffer = ' ';
            if(lSerial->IsConnected()){
                while(lSerial->ReadData(&buffer,1) != -1){
                    cout << buffer;
                }
            }
        }
        else if(lCommand == "exit" || lCommand == "quit"){
            cout << "Exiting Program..." << endl;
            break;
        }
        else{
            cout << "Invalid Command" << endl;
        }
    }

}

