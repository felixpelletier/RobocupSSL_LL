#include <boost/asio.hpp>
#ifndef SYN_SERIAL_H
#define SYN_SERIAL_H

using namespace std;

class Syn_serial{
private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;

public:
    Syn_serial(string port);
    void update_setting(string port, unsigned int baudrate);
    void write(float f1, float f2);//, float f3);

    void updateParameter(int bd,
                         int carSize,
                         boost::asio::serial_port_base::flow_control flow,
                         boost::asio::serial_port_base::parity parity,
                         boost::asio::serial_port_base::stop_bits stopBit);

};

#endif // SYN_SERIAL_H
