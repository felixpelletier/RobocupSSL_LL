#ifndef SERIALCLASS_H_INCLUDED
#define SERIALCLASS_H_INCLUDED

#define ARDUINO_WAIT_TIME 1000

#ifdef __linux__
    #include <boost/asio.hpp>
    using namespace::boost::asio;
#endif
#ifdef __WIN32__
    #include <windows.h>

#endif
#include <stdio.h>
#include <stdlib.h>

class Serial
{
    private:
        //Serial comm handler
#ifdef __WIN32__
        HANDLE hSerial;
        //Get various information about the connection
        COMSTAT status;
        //Keep track of last error
        DWORD errors;
#endif
#ifdef __linux__
        boost::asio::io_service mIO;
        boost::asio::serial_port mSerial;
#endif
        //Connection status
        bool connected;

    public:
        //Initialize Serial communication with the given COM port
        Serial(const char *portName);
        //Close the connection
        //NOTA: for some reason you can't connect again before exiting
        //the program and running it again
        ~Serial();
        //Read data in a buffer, if nbChar is greater than the
        //maximum number of bytes available, it will return only the
        //bytes available. The function return -1 when nothing could
        //be read, the number of bytes actually read.
        int ReadData(char *pBuffer, unsigned int pNbChar);
        //Writes data from a buffer through the Serial connection
        //return true on success.

        void gg(char *f1);
        bool WriteData(char *pBuffer, unsigned int pNbChar);
        //Check if we are actually connected
        bool IsConnected();


};

#endif // SERIALCLASS_H_INCLUDED
