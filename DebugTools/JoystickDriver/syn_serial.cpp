#include "syn_serial.h"
#include <iostream>
#include <sstream>

using namespace::boost::asio;

Syn_serial::Syn_serial(string port): io(), serial(io,port){

    // what baud rate do we communicate at
    serial_port_base::baud_rate baudRate(115200);
    // what caracters size is used (default is 8)
    serial_port_base::character_size carSize(8);
    // what flow control is used (default is none)
    serial_port_base::flow_control flowMod( serial_port_base::flow_control::none );
    // what parity is used (default is none)
    serial_port_base::parity parity( serial_port_base::parity::none );
    // how many stop bits are used (default is one)
    serial_port_base::stop_bits stopBit( serial_port_base::stop_bits::one );


    serial.set_option(baudRate);
    serial.set_option(carSize);
    serial.set_option(flowMod);
    serial.set_option(parity);
    serial.set_option(stopBit);
}

void Syn_serial::updateParameter(int bd, int carSize, serial_port_base::flow_control flow, serial_port_base::parity parity, serial_port_base::stop_bits stopBit){
    serial.set_option(serial_port_base::baud_rate(bd));
    serial.set_option(serial_port_base::character_size(carSize));
    serial.set_option(flow);
    serial.set_option(parity);
    serial.set_option(stopBit);
}

void Syn_serial::write(float f1, float f2){//, float f3){
    //boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size())); 0.2 = CD CC 4C 3E
    //cout << "hexfloat:\n" << std::hexfloat; //3E 4C CC CD -> lol
    //cout << s << endl;
    boost::array<float, 2> foo = {f1, f2};//, f3};
    boost::asio::write(serial, buffer(foo));
}
