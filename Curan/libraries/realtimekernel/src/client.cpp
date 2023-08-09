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

asio::io_context io_content;
asio::ip::tcp::socket* socket_pointer = nullptr;
std::array<unsigned char,10000> asio_memory_buffer;

void signal_handler(int val)
{
    if(socket_pointer!=nullptr)
        socket_pointer->shutdown(asio::ip::tcp::socket::shutdown_both);
    io_content.stop();
}

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

   while(!io_content.stopped()){
        asio::read(client_socket,asio::buffer(asio_memory_buffer),asio::transfer_exactly(sizeof(unsigned char)),ec);
        if(ec){
            std::printf("failed to send information\n terminating....\n");
            io_content.stop();
        }

        asio::read(client_socket, asio::buffer(asio_memory_buffer), asio::transfer_exactly(sizeof(unsigned char)* number_sensors), ec);
        if (ec) {
            std::printf("failed to read information\n terminating....\n");
            io_content.stop();
        }

        asio::write(client_socket,asio::buffer(asio_memory_buffer),asio::transfer_exactly(sizeof(double)),ec);
        if(ec){
            std::printf("failed to send control action\n terminating....\n");
        }
    }
}