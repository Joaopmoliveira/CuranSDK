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

asio::io_context io_context;
void signal_handler(int signal){
    io_context.stop();
};

constexpr auto maximum_delay_in_milliseconds = std::chrono::milliseconds(10);
struct Client;

void safety_shutdown(Client* control_law);

struct Client{
  asio::high_resolution_timer timer;
  bool data_sent = false;
  std::array<unsigned char,10000> allocated_memory_buffer;
  asio::io_context& context;
  asio::ip::tcp::socket sensor_socket_;
  asio::ip::tcp::socket client_socket_;

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

void do_request_sensors_acquistion(Client& ref);
void do_read_sensors_ack(Client& ref);
void do_read_sensors_updated_list(Client& ref);
void do_send_trigger_client(Client& ref);
void do_read_control_action(Client& ref);

void do_request_sensors_acquistion(Client& client) {
  client.timer.expires_from_now(maximum_delay_in_milliseconds);
  client.timer.async_wait([&](asio::error_code ec) { // handles what happens when timer ends
    if(client.data_sent){
      client.data_sent = false;
      do_request_sensors_acquistion(client);
      return ;
    }
    else{
      client.context.stop();
      return ;
    } 
  });
    asio::async_write( client.sensor_socket_,asio::buffer(client.allocated_memory_buffer),asio::transfer_exactly(message_size),
    [ &client](asio::error_code ec, size_t /*length*/) {
        if (ec) {
          client.context.stop();
          return ;
        } 
        do_read_sensors_ack(client);
  });
};

void do_read_sensors_ack(Client& client){
  asio::async_read( client.sensor_socket_, asio::buffer(client.allocated_memory_buffer.data(),sizeof(unsigned char)),asio::transfer_exactly(sizeof(unsigned char)), 
              [ &client](asio::error_code ec, size_t /*length*/) {
                  if (ec) {
                    client.context.stop();
                    return ;
                  } 
                  unpack_header_sensor_data(client.allocated_memory_buffer,client.header);
                  do_read_sensors_updated_list(client);

            });
}
void do_read_sensors_updated_list(Client& client){
  asio::async_read( client.sensor_socket_, asio::buffer(client.allocated_memory_buffer.data(), client.header.number_sensors_present),asio::transfer_exactly(client.header.number_sensors_present),
              [ &client](asio::error_code ec, size_t /*length*/) {
                  if (ec) {
                    client.context.stop();
                    return ;
                  }                   
                  unpack_body_sensor_data(client.allocated_memory_buffer,client.body,client.header);
                  for(auto && sensor_data :  client.body.buffer ){
                    std::cout << (unsigned int) sensor_data ;
                  }
                  client.bytesRead += client.header.number_sensors_present;
                  do_send_trigger_client(client);
                  
            });
}

void do_send_trigger_client(Client& client) {
    client.allocated_memory_buffer[0] = client.header.number_sensors_present; 
    std::memcpy(&(client.allocated_memory_buffer[1]), client.body.buffer.data(), client.header.number_sensors_present*sizeof(unsigned char));

    size_t message_size = sizeof(unsigned char)*(client.header.number_sensors_present+1);

    asio::async_write(client.client_socket_, asio::buffer(client.allocated_memory_buffer), asio::transfer_exactly(message_size),
        [&client](asio::error_code ec, size_t /*length*/) {
            if (ec) {
                client.context.stop();
                return;
            }
            do_read_control_action(client);
        });

}

void do_read_control_action(Client& client){
  asio::async_read( client.client_socket_, asio::buffer(client.allocated_memory_buffer.data(),sizeof(double)),asio::transfer_exactly(sizeof(double)), 
              [ &client](asio::error_code ec, size_t /*length*/) {
                  if (ec) {
                    client.context.stop();
                    return ;
                  } 
                  memcpy(&control_action, client.allocated_memory_buffer.data(), sizeof(double));
                  client.data_sent = true;

            });
}


int main(int argc, char* argv[])
{
  std::signal(SIGINT, signal_handler);

  asio::ip::tcp::socket sensor_socket(io_context);
  asio::ip::tcp::resolver sensor_resolver(io_context);
  asio::connect(sensor_socket, sensor_resolver.resolve("localhost","50000"));

  asio::ip::tcp::socket client_socket(io_context);
  asio::ip::tcp::resolver client_resolver(io_context);
  asio::connect(client_socket, client_resolver.resolve("localhost","50001"));

  Client watchgod{io_context,std::move(sensor_socket),std::move(client_socket) };
  
  do_request_sensors_acquistion(watchgod);

  io_context.run();
  return 0;
};