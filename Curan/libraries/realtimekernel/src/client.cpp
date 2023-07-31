#include <array>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <csignal>
#include <thread>
#include <sstream>
#include <asio.hpp>
#include "header_acessor.h"
#include <cmath>
#include <type_traits>

asio::io_context io_content;
asio::ip::tcp::socket* client_socket_hiipi = nullptr;
Sincronizer sincronizer;

void signal_handler(int val)
{
    sincronizer.stop();
    if(client_socket_hiipi!=nullptr)
        client_socket_hiipi->shutdown(asio::ip::tcp::socket::shutdown_both);
    io_content.stop();
}

int main(){
    std::signal(SIGINT,signal_handler);

    unsigned int port = 50001;
    asio::ip::tcp::endpoint endpoit(asio::ip::tcp::v4(), port);
    asio::ip::tcp::acceptor acceptor(io_content,endpoit);

    std::array<unsigned char,10> allocated_memory_buffer;

    asio::error_code ec;
    asio::ip::tcp::socket client_socket = acceptor.accept();
    if (ec){
        std::cout << "failed to run, terminating....\n";
        std::cout << ec.message() << std::endl;
        return 1;
    };
    client_socket_hiipi = &client_socket;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    try{
   for(size_t counter = 0;!sincronizer.is_stoped(); ++counter){
        asio::read(client_socket,asio::buffer(allocated_memory_buffer),asio::transfer_exactly(sizeof(unsigned char)),ec);
        if(ec){
            std::printf("failed to send information\n terminating....\n");
            return 1;
        }

        uint8_t number_sensors = allocated_memory_buffer[0];
        asio::read(client_socket, asio::buffer(allocated_memory_buffer), asio::transfer_exactly(sizeof(unsigned char)* number_sensors), ec);
        if (ec) {
            std::printf("failed to read information\n terminating....\n");
            return 1;
        }
        
        gps_reading g_read;
        for (int sensor_element_i = 0; sensor_element_i < number_sensors; ++sensor_element_i) {
            copy_from_shared_memory_to_gps_reading(shared_memory->get_shared_memory_address(), g_read);
        }
        std::cout << g_read.angular_velocity[2] << std::endl;

        double control_action = g_read.angular_velocity[2];
        asio::write(client_socket,asio::buffer(&control_action,sizeof(double)),asio::transfer_exactly(sizeof(double)),ec);
        if(ec){
            std::printf("failed to send control action\n terminating....\n");
        }
    }
    }catch(...){
        std::cout << "failure was detected in either communication or shared memory operation\n";
    }
}