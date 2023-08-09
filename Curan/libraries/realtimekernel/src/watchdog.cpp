#include <asio.hpp>
#include <ctime>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <utility>
#include <array>
#include <csignal>
#include "watchdogmessage.h"

asio::io_context io_context;
constexpr watchdog_message_layout message_layout;
constexpr size_t watchdog_message_size = message_layout.image_reading_present_address+message_layout.image_reading_present_size;
void signal_handler(int signal){
    io_context.stop();
};

constexpr auto maximum_delay_in_milliseconds = std::chrono::milliseconds(10);
struct Client;

void safety_shutdown(Client* control_law);

struct Client{
  asio::high_resolution_timer timer;
  bool data_sent = true;
  std::array<unsigned char,watchdog_message_size> allocated_memory_buffer;
  asio::io_context& context;
  asio::ip::tcp::socket sensor_socket_;
  asio::ip::tcp::socket client_socket_;
  watchdog_message message;

  explicit Client(asio::io_context& in_context,
                    asio::ip::tcp::socket&& sensor_socket,
                    asio::ip::tcp::socket&& client_socket
                                                      ) : context{in_context} ,
                                                         sensor_socket_{std::move(sensor_socket)},
                                                        client_socket_{ std::move(client_socket) },
                                                         timer{in_context}{}

  Client(const Client & copyclient) = delete;

  Client(Client && client) = delete;

  ~Client(){
    timer.cancel();
    safety_shutdown(this);
  }
};

void safety_shutdown(Client* control_law){
   std::cout << "terminating with safety stop because something went wrong (shutting down controller)" << std::endl;
   std::cout.flush();
}

void write_control(Client& ref);
void request_sensors_acquistion(Client& ref);
void read_sensors_acknowledgment(Client& ref);
void warn_client_of_readings(Client& ref);
void read_client_acknowledgment(Client& ref);

void write_control(Client& client){
  client.timer.expires_from_now(maximum_delay_in_milliseconds);
  client.timer.async_wait([&](asio::error_code ec) { // handles what happens when timer ends
    if(client.data_sent){
      client.data_sent = false;
      //here we should do some kind of control
      write_control(client);
      return ;
    }
    else{
      client.context.stop();
      return ;
    } 
  });
  request_sensors_acquistion(client);
}

void request_sensors_acquistion(Client& client) {
  copy_from_watchdog_message_to_shared_memory(client.allocated_memory_buffer.data(),client.message);
  asio::async_write( client.sensor_socket_,asio::buffer(client.allocated_memory_buffer),asio::transfer_exactly(watchdog_message_size),
    [ &client](asio::error_code ec, size_t /*length*/) {
        if (ec) {
          client.context.stop();
          return ;
        }
        read_sensors_acknowledgment(client);
  });
};

void read_sensors_acknowledgment(Client& client){
  asio::async_read( client.sensor_socket_, asio::buffer(client.allocated_memory_buffer),asio::transfer_exactly(watchdog_message_size), 
    [ &client](asio::error_code ec, size_t /*length*/) {
      if (ec) {
        client.context.stop();
        return ;
      } 
      copy_from_shared_memory_to_watchdog_message(client.allocated_memory_buffer.data(),client.message);
      warn_client_of_readings(client);
  });
}
void warn_client_of_readings(Client& client){
  asio::async_write( client.sensor_socket_,asio::buffer(client.allocated_memory_buffer),asio::transfer_exactly(watchdog_message_size),
    [ &client](asio::error_code ec, size_t /*length*/) {
        if (ec) {
          client.context.stop();
          return ;
        }
        read_client_acknowledgment(client);
  });
}

void read_client_acknowledgment(Client& client) {
  asio::async_read( client.sensor_socket_, asio::buffer(client.allocated_memory_buffer),asio::transfer_exactly(watchdog_message_size), 
    [ &client](asio::error_code ec, size_t /*length*/) {
      if (ec) {
        client.context.stop();
        return ;
      } 
      copy_from_shared_memory_to_watchdog_message(client.allocated_memory_buffer.data(),client.message);
      client.data_sent = true;
  });
}

int main(int argc, char* argv[])
{
  std::chrono::time_point currently = std::chrono::time_point_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now()
  );
  std::chrono::duration millis_since_utc_epoch = currently.time_since_epoch();

  std::signal(SIGINT, signal_handler);

  asio::ip::tcp::socket sensor_socket(io_context);
  asio::ip::tcp::resolver sensor_resolver(io_context);
  asio::connect(sensor_socket, sensor_resolver.resolve("localhost","50000"));

  asio::ip::tcp::socket client_socket(io_context);
  asio::ip::tcp::resolver client_resolver(io_context);
  asio::connect(client_socket, client_resolver.resolve("localhost","50001"));

  Client watchgod{io_context,std::move(sensor_socket),std::move(client_socket) };
  
  write_control(watchgod);

  io_context.run();
  return 0;
};