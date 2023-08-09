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
#include "header_acessor.h"
#include "watchdogmessage.h"

asio::io_context io_content;
constexpr watchdog_message_layout message_layout;
constexpr size_t watchdog_message_size = message_layout.image_reading_present_address+message_layout.image_reading_present_size;
asio::ip::tcp::socket* socket_pointer = nullptr;
std::array<unsigned char,watchdog_message_size> asio_memory_buffer;

auto shared_memory = SharedMemoryAccessor::create();

void signal_handler(int val){
    if(socket_pointer!=nullptr)
        socket_pointer->shutdown(asio::ip::tcp::socket::shutdown_both);
    io_content.stop();
}

void render_scene(){

};

int main(){
    std::signal(SIGINT,signal_handler);

    unsigned int port = 50001;
    asio::ip::tcp::endpoint endpoit(asio::ip::tcp::v4(), port);
    asio::ip::tcp::acceptor acceptor(io_content,endpoit);

    asio::error_code ec;
    asio::ip::tcp::socket client_socket = acceptor.accept();
    if (ec){
        std::cout << "failed to run, terminating....\n";
        std::cout << ec.message() << std::endl;
        return 1;
    };
    socket_pointer = &client_socket;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    watchdog_message message;
   while(!io_content.stopped()){
        asio::read(client_socket, asio::buffer(asio_memory_buffer), asio::transfer_exactly(watchdog_message_size), ec);
        if (ec) {
            std::printf("failed to read information\n terminating....\n");
            io_content.stop();
        }
        copy_from_memory_to_watchdog_message(asio_memory_buffer.data(),message);

        //now that 
        if(message.gps_reading_present);
        
        if(message.image_reading_present);  

        copy_from_watchdog_message_to_memory(asio_memory_buffer.data(),message);
        asio::write(client_socket,asio::buffer(asio_memory_buffer),asio::transfer_exactly(watchdog_message_size),ec);
        if(ec){
            std::printf("failed to send control action\n terminating....\n");
        }
    }
}