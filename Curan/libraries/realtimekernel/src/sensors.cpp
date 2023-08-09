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

void gps_readings_thread(std::atomic<gps_reading>& global_shared_gps_reading,std::atomic<bool> continue_running){
    gps_reading reading; 
    reading.counter= 0;
    while(continue_running.load()){
         //should write to shared memory
        reading.acceleration[0] = system_state_packet.body_acceleration[0];
        reading.acceleration[1] = system_state_packet.body_acceleration[1];
        reading.acceleration[2] = system_state_packet.body_acceleration[2];
        reading.angular_velocity[0] = system_state_packet.angular_velocity[0];
        reading.angular_velocity[1] = system_state_packet.angular_velocity[1];
        reading.angular_velocity[2] = system_state_packet.angular_velocity[2];
        reading.counter++;
        reading.gforce = system_state_packet.g_force;
        reading.height=system_state_packet.height;
        reading.latitude = system_state_packet.latitude;
        reading.longitude= system_state_packet.longitude;
        reading.orientation[0] = system_state_packet.orientation[0];
        reading.orientation[1] = system_state_packet.orientation[1];
        reading.orientation[2] = system_state_packet.orientation[2];
        reading.standard_deviation[0] = system_state_packet.standard_deviation[0];
        reading.standard_deviation[1] = system_state_packet.standard_deviation[1];
        reading.standard_deviation[2] = system_state_packet.standard_deviation[2];
        reading.velocity[0]= system_state_packet.velocity[0];
        reading.velocity[1]= system_state_packet.velocity[1];
        reading.velocity[2]= system_state_packet.velocity[2];

        global_shared_gps_reading.store(reading);
    }
}

void image_reading_thread(std::vector<unsigned char>& image_memory_blob,std::mutex& camera_reading_mutex,grayscale_image_1& global_shared_camera_reading,std::atomic<bool> continue_running){
    
    while(continue_running.load()){
        
    }
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

    std::thread gps_sensor{[&](){gps_readings_thread();}};
    std::thread image_sensor{[&](){image_reading_thread();}};

    watchdog_message message;
   for(size_t counter = 0;!io_context.stopped(); ++counter){
        asio::read(client_socket,asio::buffer(asio_memory_buffer),asio::transfer_exactly(watchdog_message_size),ec);
        if(ec){
            std::printf("failed to send information\n terminating....\n");
            io_context.stop();
        } 
        copy_from_memory_to_watchdog_message(asio_memory_buffer.data(),message);

        //now that 

        copy_from_watchdog_message_to_memory(asio_memory_buffer.data(),message);
        asio::write(client_socket,asio::buffer(asio_memory_buffer),asio::transfer_exactly(watchdog_message_size),ec);
        if(ec){
            std::printf("failed to send information\n terminating....\n");
            io_context.stop();
        }
        
    }
    gps_sensor.join();
    image_sensor.join();
    }catch(...){
        std::cout << "failure was detected in either communication or shared memory operation\n";
    }
    
}