#include <asio.hpp>
#include <iostream>

constexpr size_t maximum_length_of_message = 1000; 

int main(int argc, char* argv[]){
try{
    std::string serial_connection_name = std::string(CURAN_SERIAL_PORT);
    if(serial_connection_name.size()==0){
        if(argc<2){
            std::cout << "To use this service please provide the port number of the serial connection \neg. windows will be COM ports, linux will use /dev/ttyS* or /dev/ttyUSB*, etc\n" ;
            return 1;
        }
        serial_connection_name = std::string(argv[1]);
    }
    
    asio::io_context context;
    asio::serial_port serial(context);
    serial.open(serial_connection_name);

    std::atomic<bool> value = true;
    auto stopper = [&value](){
        std::string input;
        std::cout << "Enter Message: ";
        std::cin >> input;
        value.store(false);
    };
    std::thread to_stop{stopper};

    char data[maximum_length_of_message];

    for (;value.load();) {
        // read bytes from the serial port
        // asio::read will read bytes until the buffer is filled
        size_t nread = asio::read(
            serial, asio::buffer(data, 2)
        );
        std::string message(data, nread);
        std::cout << "Recieved: (";
        std::cout << message << ")\n";
    }
    serial.close();
    to_stop.join();
    return 0; 
} catch(std::exception & e){
    std::cout << "exception thrown while reading the serial stream\n";
    return 1;
}

}