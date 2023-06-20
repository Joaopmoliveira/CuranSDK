#include <asio.hpp>
#include <iostream>

constexpr size_t maximum_length_of_message = 1000; 

int main(int argc, char* argv[]){
    if(argc<2){
        std::cout << "To use this service please provide the port number of the serial connection \neg. windows will be COM ports, linux will use /dev/ttyS* or /dev/ttyUSB*, etc\n" ;
    }

    std::string serial_connection_name = std::string(CURAN_SERIAL_PORT);
    if(serial_connection_name.size()==0)
        serial_connection_name = std::string(argv[1]);
    
    asio::io_context context;
    asio::serial_port serial(context);
    serial.open(serial_connection_name);
    for (;;) {
        // get a string from the user, sentiel is exit
        std::string input;
        std::cout << "Enter Message: ";
        std::cin >> input;

        if (input == "exit") break;

        // write to the port
        // asio::write guarantees that the entire buffer is written to the serial port
        asio::write(serial, asio::buffer(input));

        char data[maximum_length_of_message];

        // read bytes from the serial port
        // asio::read will read bytes until the buffer is filled
        size_t nread = asio::read(
            serial, asio::buffer(data, input.length())
        );

        std::string message(data, nread);

        std::cout << "Recieved: ";
        std::cout << message << std::endl;
    }
    serial.close();
    return 0; 
}