#include "command.h"

void printHelp(){
    cout << "Valid Commands:" << endl   << endl;
    cout << endl << " - connect (cn)"   << endl;
    cout << " - param (pr)"             << endl;
    cout << " - sendbuffer (sb) "       << endl;
    cout << " - printPacket (pp)"       << endl;
    cout << " - sendPacket (sp)"        << endl;
    cout << " - quit (exit)"            << endl;
}

Serial* connectToPort(){
    string lPort;
    cout << "Port: ";
    cin >> lPort;
#ifdef __WIN32__
    string lfilePath = "\\\\.\\COM"+ lPort;
#endif
#ifdef __linux__
    string lfilePath = "/dev/"+ lPort;
#endif

    Serial * pSerial = new Serial(lfilePath.c_str());
    if(!pSerial->IsConnected()){
        cout << "Unable to connect to Serial Port :" + lPort << endl;
        return NULL;
    }
    else{
        cout << "Connection established" << endl;
        return pSerial;
    }
}

void printSpeedPacket(float iX, float iY, float iTheta, int iId){
    Packer lTemp(NULL);
    lTemp.createSpeedCommand(iX,iY,iTheta,iId);
    lTemp.printPacket();
}

void sendPacket(float iHz, int iQuantity,Serial *iSerial){
    Packer lTemp(iSerial);
    float iDelayms = (1/iHz)*1000;
    for(int i = 1; i <= iQuantity; ++i){
        lTemp.createSpeedCommand(2,1,0,1);  //toDO random packet
        lTemp.sendPacket();

#ifdef __WIN32__
        Sleep(iDelayms);
#endif
#ifdef __linux__
        usleep(iDelayms*1000);
#endif
    }
}
