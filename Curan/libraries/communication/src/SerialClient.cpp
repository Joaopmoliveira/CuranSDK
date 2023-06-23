#include "communication/SerialClient.h"

namespace curan {
namespace communication {

SerialClient::SerialClient(Info& info) : serial(info.service){
    serial.set_option(asio::serial_port_base::baud_rate(info.baud_rate));
    serial.set_option(asio::serial_port_base::character_size(8 /* data bits */));
    // TODO : I dont know if I need these options in the future or not....
    //serial.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    //serial.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
    serial.open(info.port_name);
}

std::string SerialClient::read(size_t n){
        asio::streambuf input_buffer;
        size_t nread = asio::read_until(
            serial, input_buffer, 'e'
        );
        std::istream is(&input_buffer);
        std::string line;
        std::getline(is, line);
        return line;
}

void SerialClient::write(const std::string& s){
    asio::write(serial,asio::buffer(s.data(),s.size()));
}

void SerialClient::close(){
    serial.close();
}

}
}