#include <array>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <csignal>
#include <thread>
#include <sstream>
#include <asio.hpp>
#include <cmath>
#include <type_traits>
#include "header_creator.h"
#include "watchdogmessage.h"

asio::io_context io_context;
asio::ip::tcp::socket* socket_pointer = nullptr;
constexpr watchdog_message_layout message_layout;
constexpr size_t watchdog_message_size = message_layout.image_reading_present_address+message_layout.image_reading_present_size;
std::array<unsigned char,watchdog_message_size> asio_memory_buffer;

auto shared_memory = SharedMemoryCreator::create();

void sensor_thread(){

}

void signal_handler(int val)
{
    if(socket_pointer!=nullptr)
        socket_pointer->shutdown(asio::ip::tcp::socket::shutdown_both);
    io_context.stop();
}

int main(){
    try{
    std::signal(SIGINT,signal_handler);
    unsigned int port = 50000;
    asio::ip::tcp::endpoint endpoit(asio::ip::tcp::v4(), port);
    asio::ip::tcp::acceptor acceptor(io_context,endpoit);

    asio::error_code ec;
    asio::ip::tcp::socket client_socket = acceptor.accept();
    if (ec){
        std::cout << "failed to run, terminating....\n";
        std::cout << ec.message() << std::endl;
        return 1;
    };

    socket_pointer = &client_socket;

    std::thread sensor{[&](){sensor_thread();}};

   for(size_t counter = 0;!io_context.stopped(); ++counter){
        asio::read(client_socket,asio::buffer(asio_memory_buffer),asio::transfer_exactly(sizeof(unsigned char)),ec);
        if(ec){
            std::printf("failed to send information\n terminating....\n");
            io_context.stop();
        } 

        //now that 

        asio::write(client_socket,asio::buffer(asio_memory_buffer),asio::transfer_exactly(watchdog_message_size),ec);
        if(ec){
            std::printf("failed to send information\n terminating....\n");
            io_context.stop();
        }
    }
    sensor.join();
    }catch(...){
        std::cout << "failure was detected in either communication or shared memory operation\n";
    }
    
}