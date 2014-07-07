//version 1.0
#include "packer.h"

Packer::Packer(Serial * pSerial){
    mSerial = pSerial;
}

unsigned char Packer::FLAGBYTE = 0x7E;
unsigned char Packer::ESCAPEBYTE = 0x7D;

void Packer::createSpeedCommand(float iX, float iY, float iTheta, unsigned char iId){
    std::queue<unsigned char> emptyQueue;
    std::swap( mPacket, emptyQueue);

    this->insertFlagByteInPacket();
    mPacket.push(iId);
    mPacket.push(char(1));
    this->insertFloatInPacket(iX);
    this->insertFloatInPacket(iY);
    this->insertFloatInPacket(iTheta);
    this->insertFlagByteInPacket();
}

void Packer::createSetPidCommand(float iP, float iI, float iD, unsigned char iId){
}

void Packer::sendPacket(){
    int lSize = this->mPacket.size();
    char lBuffer[lSize + 1];
    for(int i = 0; i < lSize; ++i){
        lBuffer[i] = mPacket.front();
        mPacket.pop();
    }
    if(mSerial->IsConnected()){
        mSerial->WriteData(lBuffer,lSize + 1);
    }
    else{
        cout << "Serial not connected" << endl;
    }

}


void Packer::insertFloatInPacket(float iData){
    mDataConverter.floatValue = iData;
    for(int i = 0; i < 4; ++i){
        if(mDataConverter.charValues[i] == this->FLAGBYTE || mDataConverter.charValues[i] == this->ESCAPEBYTE){
            mPacket.push(this->ESCAPEBYTE);
        }
        mPacket.push(mDataConverter.charValues[i]);
    }
}


void Packer::insertIntInPacket(int iData){
    mDataConverter.intValue = iData;
    for(int i = 0; i < 4; ++i){
        if(mDataConverter.charValues[i] == this->FLAGBYTE || mDataConverter.charValues[i] == this->ESCAPEBYTE){
            mPacket.push(this->ESCAPEBYTE);
        }
        mPacket.push(mDataConverter.charValues[i]);
    }
}

void Packer::insertFlagByteInPacket(){
    mPacket.push(this->FLAGBYTE);
}

void Packer::printPacket(){
    std::queue<unsigned char> lCopy(mPacket);
    int lSize = lCopy.size();
    std::cout << "Printing Packet of length : " << lSize << std::endl;
    std::cout << "---------------------------------------------- " << std::endl;
    for(int i = 0; i < lSize; ++i){
        printf("%x ",lCopy.front());
        lCopy.pop();
    }
    std::cout << std::endl << "---------------------------------------------- " << std::endl;
}
