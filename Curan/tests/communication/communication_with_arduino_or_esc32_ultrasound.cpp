#include <asio.hpp>
#include <iostream>

constexpr size_t maximum_length_of_message = 4; 

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
    auto stopper = [&value,&serial](){
        std::string input;
        std::cin >> input;
        value.store(false);
        serial.close();
    };
    std::thread to_stop{stopper};

 
    char to_send = 10;
    size_t nread = 0;
    for (int counter = 0;value.load();++counter) {
        asio::write(serial,asio::buffer(&to_send,1));
        asio::streambuf input_buffer;
        nread = asio::read_until(
            serial, input_buffer, 'e'
        );
        std::istream is(&input_buffer);
        std::string line;
        std::getline(is, line);
        std::cout << "Received: (" << line << ") cm pressed: " << counter << "\n";
    }
    to_stop.join();
    return 0; 
} catch(std::exception & e){
    std::cout << "exception thrown while reading the serial stream\n";
    return 1;
}

}