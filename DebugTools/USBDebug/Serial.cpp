#include "Serial.h"

#ifdef __WIN32__
Serial::Serial(const char *portName):connected(false)
{
    //Try to connect to the given port throuh CreateFile
    this->hSerial = CreateFile(portName,
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL);

    //Check if the connection was successfull
    if(this->hSerial==INVALID_HANDLE_VALUE)
    {
        //If not success full display an Error
        if(GetLastError()==ERROR_FILE_NOT_FOUND){

            //Print Error if neccessary
            printf("ERROR: Handle was not attached. Reason: %s not available.\n", portName);

        }
        else
        {
            printf("ERROR!!!");
        }
    }
    else
    {
        //If connected we try to set the comm parameters
        DCB dcbSerialParams = {0};

        //Try to get the current
        if (!GetCommState(this->hSerial, &dcbSerialParams))
        {
            //If impossible, show an error
            printf("failed to get current serial parameters!");
        }
        else
        {
            //Define serial connection parameters for the arduino board
            dcbSerialParams.BaudRate=CBR_115200;
            dcbSerialParams.ByteSize=8;
            dcbSerialParams.StopBits=ONESTOPBIT;
            dcbSerialParams.Parity=NOPARITY;

             //Set the parameters and check for their proper application
             if(!SetCommState(hSerial, &dcbSerialParams))
             {
                printf("ALERT: Could not set Serial Port parameters");
             }
             else
             {
                 //If everything went fine we're connected
                 this->connected = true;
                 //We wait 2s as the arduino board will be reseting
                 Sleep(ARDUINO_WAIT_TIME);
             }
        }
    }
#endif
#ifdef __linux__
Serial::Serial(const char *portName):connected(true),mIO(),mSerial(mIO,std::string(portName))
    {
    // what baud rate do we communicate at
    serial_port_base::baud_rate lBaudRate(115200);
    // what caracters size is used (default is 8)
    serial_port_base::character_size lCarSize( 8 );
    // what flow control is used (default is none)
    serial_port_base::flow_control lFlowMod( serial_port_base::flow_control::none );
    // what parity is used (default is none)
    serial_port_base::parity lParity( serial_port_base::parity::none );
    // how many stop bits are used (default is one)
    serial_port_base::stop_bits lStopBit( serial_port_base::stop_bits::one );


    mSerial.set_option(lBaudRate);
    mSerial.set_option(lCarSize);
    mSerial.set_option(lFlowMod);
    mSerial.set_option(lParity);
    mSerial.set_option(lStopBit);
#endif

}

Serial::~Serial()
{
    //Check if we are connected before trying to disconnect
    if(this->connected)
    {
        //We're no longer connected
        this->connected = false;
        //Close the serial handler
#ifdef __WIN32__
        CloseHandle(this->hSerial);
#endif
    }
}

int Serial::ReadData(char *pBuffer, unsigned int pNbChar)
{
#ifdef __WIN32__
    //Number of bytes we'll have read
    DWORD bytesRead;
    //Number of bytes we'll really ask to read
    unsigned int toRead;

    //Use the ClearCommError function to get status info on the Serial port
    ClearCommError(this->hSerial, &this->errors, &this->status);

    //Check if there is something to read
    if(this->status.cbInQue>0)
    {
        //If there is we check if there is enough data to read the required number
        //of characters, if not we'll read only the available characters to prevent
        //locking of the application.
        if(this->status.cbInQue>pNbChar)
        {
            toRead = pNbChar;
        }
        else
        {
            toRead = this->status.cbInQue;
        }

        //Try to read the require number of chars, and return the number of read bytes on success
        if(ReadFile(this->hSerial, pBuffer, toRead, &bytesRead, NULL) && bytesRead != 0)
        {
            return bytesRead;
        }

    }
    //If nothing has been read, or that an error was detected return -1
    return -1;
#endif
#ifdef __linux__
    boost::asio::read(mSerial,buffer(pBuffer,pNbChar));
    return 1;
#endif

}
void Serial::gg(char *f1){
    char c;

    while((c= *f1++)!=0){
        char buf;
        buf=c;
        this->WriteData(&buf, 1);
    }
}


bool Serial::WriteData(char *pBuffer, unsigned int pNbChar)
{
#ifdef __WIN32__
    DWORD bytesSend;

    //Try to write the buffer on the Serial port
    if(!WriteFile(this->hSerial, (void *)pBuffer, pNbChar, &bytesSend, 0))
    {
        //In case it don't work get comm error and return false
        ClearCommError(this->hSerial, &this->errors, &this->status);

        return false;
    }
    else
        return true;
#endif
#ifdef __linux__
    boost::asio::write(mSerial, buffer(pBuffer,pNbChar));
    return true;
#endif
}

bool Serial::IsConnected()
{
    //Simply return the connection status
    return this->connected;
}
