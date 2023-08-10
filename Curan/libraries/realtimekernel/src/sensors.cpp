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
#include <random>

asio::io_context io_context;
asio::ip::tcp::socket* socket_pointer = nullptr;
constexpr watchdog_message_layout message_layout;
constexpr size_t watchdog_message_size = message_layout.image_reading_present_address+message_layout.image_reading_present_size;
std::array<unsigned char,watchdog_message_size> asio_memory_buffer;

auto shared_memory = SharedMemoryCreator::create();

void gps_readings_thread(std::atomic<gps_reading>& global_shared_gps_reading,std::atomic<bool>& continue_running){
    gps_reading reading; 
    reading.counter= 0;
    double time = 0.0;
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(-0.05, 0.05);
    
    while(continue_running.load()){
         //should write to shared memory
        double phase_offset = 0.0;
        reading.acceleration[0] = std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.acceleration[1] = std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.acceleration[2] = std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.angular_velocity[0] = std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.angular_velocity[1] = std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.angular_velocity[2] = std::sin(time+phase_offset)+dis(gen);
        reading.counter++;
        phase_offset += 0.4;
        reading.gforce = std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.height= std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.latitude = std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.longitude= std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.orientation[0] = std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.orientation[1] = std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.orientation[2] = std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.standard_deviation[0] = std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.standard_deviation[1] = std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.standard_deviation[2] = std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.velocity[0]= std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.velocity[1]= std::sin(time+phase_offset)+dis(gen);
        phase_offset += 0.4;
        reading.velocity[2]= std::sin(time+phase_offset)+dis(gen);

        global_shared_gps_reading.store(reading);
        time += 0.01;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void image_reading_thread(std::vector<unsigned char>& image_memory_blob,std::mutex& camera_reading_mutex,grayscale_image_1& global_shared_camera_reading,std::atomic<bool>& continue_running){   
    std::vector<unsigned char> local_blob;
    local_blob.resize(image_memory_blob.size());
    while(continue_running.load()){
        //we need to lock everytime we want to manipulate the image_memory_blob adresss
        {
            std::lock_guard<std::mutex> lock(camera_reading_mutex);
            ++global_shared_camera_reading.counter;
            std::memcpy(image_memory_blob.data(), local_blob.data(), image_memory_blob.size());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
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

    constexpr grayscale_image_1_layout layout;
    std::vector<unsigned char> image_memory_blob;
    std::mutex camera_reading_mutex;
    grayscale_image_1 global_shared_camera_reading;
    image_memory_blob.resize(layout.data_size);
    {
        std::lock_guard<std::mutex> g{camera_reading_mutex};
        global_shared_camera_reading.counter = 1;
        global_shared_camera_reading.data = image_memory_blob.data();
    }
    std::atomic<gps_reading> global_shared_gps_reading;
    std::atomic<bool> boolean_flag_of_gps_reader = true;
    std::thread gps_sensor{[&](){gps_readings_thread(global_shared_gps_reading,boolean_flag_of_gps_reader);}};

    std::atomic<bool> boolean_flag_of_image_reader = true;
    std::thread image_sensor{[&](){image_reading_thread(image_memory_blob,camera_reading_mutex,global_shared_camera_reading,boolean_flag_of_image_reader);}};

    watchdog_message message;
   for(size_t counter = 0;!io_context.stopped(); ++counter){
        asio::read(client_socket,asio::buffer(asio_memory_buffer),asio::transfer_exactly(watchdog_message_size),ec);
        if(ec){
            std::printf("failed to send information\n terminating....\n");
            io_context.stop();
            break;
        } 
        copy_from_memory_to_watchdog_message(asio_memory_buffer.data(),message);
        std::chrono::time_point currently = std::chrono::time_point_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now()
        );
        std::chrono::duration millis_since_utc_epoch = currently.time_since_epoch();
        message.sensors_receive_timestamp = millis_since_utc_epoch.count();
        if(counter % 5 == 0){
            const auto local_copy = global_shared_gps_reading.load();
            copy_from_gps_reading_to_shared_memory(shared_memory->get_shared_memory_address(),local_copy);
            {
                std::lock_guard<std::mutex> g{camera_reading_mutex};
                copy_from_grayscale_image_1_to_shared_memory(shared_memory->get_shared_memory_address(),global_shared_camera_reading);
            }
            message.image_reading_present = true;
            message.gps_reading_present = true;
        } else {
            auto local_copy = global_shared_gps_reading.load();
            copy_from_gps_reading_to_shared_memory(shared_memory->get_shared_memory_address(),local_copy);
            message.image_reading_present = false;
            message.gps_reading_present = true;
        }
        currently = std::chrono::time_point_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now()
        );
        millis_since_utc_epoch = currently.time_since_epoch();
        message.sensors_send_timestamp = millis_since_utc_epoch.count();
        copy_from_watchdog_message_to_memory(asio_memory_buffer.data(),message);
        asio::write(client_socket,asio::buffer(asio_memory_buffer),asio::transfer_exactly(watchdog_message_size),ec);
        if(ec){
            std::printf("failed to send information\n terminating....\n");
            io_context.stop();
            break;
        }
        
    }
    std::raise(SIGINT);
    boolean_flag_of_gps_reader = false;
    boolean_flag_of_image_reader = false;
    gps_sensor.join();
    image_sensor.join();
    }catch(...){
        std::cout << "failure was detected in either communication or shared memory operation\n";
    }
    
}