#include <array>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <csignal>
#include <thread>
#include <sstream>
#include <asio.hpp>
#include "header_creator.h"
#include <cmath>
#include <type_traits>

#include "read_gnss_compass.h"
#include <librealsense2/rs.hpp> 

asio::io_context io_context;
asio::ip::tcp::socket* client_socket_hiipi = nullptr;

std::atomic<gps_reading> global_shared_gps_reading = gps_reading{};

std::mutex camera_reading_mutex;
grayscale_image_1 global_shared_camera_reading;
std::vector<unsigned char> image_memory_blob;

auto shared_memory = SharedMemoryCreator::create();

void camera_reader(){
try{

    rs2::context ctx;      
    std::vector<rs2::pipeline>            pipelines;

    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices())
    {
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    }   

    for (auto&& serial : serials)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        rs2::pipeline_profile selection = pipe.start(cfg);
        pipelines.emplace_back(pipe);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    while(!io_context.stopped()){
        for (auto &&pipe : pipelines)
        {
            rs2::frameset fs; //stores acquired frame prom specified pipe
            if (pipe.poll_for_frames(&fs)) //non-blocking
            {   
                rs2::depth_frame depth = fs.get_depth_frame();
                {
                    std::lock_guard<std::mutex> lock(camera_reading_mutex);
                    global_shared_camera_reading.counter = (int)fs.get_frame_number();
                    std::memcpy(image_memory_blob.data(), depth.get_data(), image_memory_blob.size());
                }
            }
        }

    }
} catch(...){
    std::printf("failure camera thread");
}
}

void gps_reader(){
try{
    std::array<std::string,2> ip_ad_port = {"10.16.232.83","16718"};

    an_decoder_t an_decoder;
	an_packet_t* an_packet;
	
	system_state_packet_t system_state_packet;
	raw_sensors_packet_t raw_sensors_packet;
	gps_reading reading; //shm memory
    reading.counter= 0;
	FILE* anpp_log_file;
	
	char filename[64];
	time_t rawtime;
	struct tm* timeinfo;
	int write_counter = 0;
	int bytes_received;

	struct addrinfo hints = {0}, *addresses;
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	if(getaddrinfo(ip_ad_port[0].data(), ip_ad_port[1].data(), &hints, &addresses))
	{
		printf("Failure resolving hostname %s, port %s\n", ip_ad_port[0].data(), ip_ad_port[1].data());
		exit(EXIT_FAILURE);
	}

	if((socket_fd = socket(addresses->ai_family, addresses->ai_socktype, addresses->ai_protocol)) < 0)
	{
		printf("Socket creation error \n");
		freeaddrinfo(addresses);
		exit(EXIT_FAILURE);
	}

	if(connect(socket_fd, addresses->ai_addr, addresses->ai_addrlen) < 0)
	{
		printf("Socket connection failed \n");
		freeaddrinfo(addresses);
		exit(EXIT_FAILURE); // TODO: remove
	}
	freeaddrinfo(addresses);
	
	an_decoder_initialise(&an_decoder);
	
	while(!io_context.stopped())
	{
		if((bytes_received = receive(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
		{
			an_decoder_increment(&an_decoder, bytes_received);
			
			while((an_packet = an_packet_decode(&an_decoder)) != NULL)
			{
				if(an_packet->id == packet_id_system_state)
				{
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{

					}
				}
				else if(an_packet->id == packet_id_raw_sensors)
				{
					if(decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0)
					{

					}
				}
				an_packet_free(&an_packet);
			}
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
    } catch(...){
        std::printf("failure gsp thread");
    }
    
    }

void signal_handler(int val)
{
    if(client_socket_hiipi!=nullptr)
        client_socket_hiipi->shutdown(asio::ip::tcp::socket::shutdown_both);
    io_context.stop();
}

int main(){
    std::signal(SIGINT,signal_handler);
    unsigned int port = 50000;
    asio::ip::tcp::endpoint endpoit(asio::ip::tcp::v4(), port);
    asio::ip::tcp::acceptor acceptor(io_context,endpoit);

    std::array<unsigned char,10> allocated_memory_buffer;

    asio::error_code ec;
    asio::ip::tcp::socket client_socket = acceptor.accept();
    if (ec){
        std::cout << "failed to run, terminating....\n";
        std::cout << ec.message() << std::endl;
        return 1;
    };

    client_socket_hiipi = &client_socket;
    constexpr grayscale_image_1_layout layout;

    image_memory_blob.resize(layout.data_size);
    {
        std::lock_guard<std::mutex> g{camera_reading_mutex};
        global_shared_camera_reading.counter = 1;
        global_shared_camera_reading.data = image_memory_blob.data();
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    std::thread camera_thread{[&](){camera_reader();}};
    std::thread gps_thread{[&](){gps_reader();}};
    try{
   for(size_t counter = 0;!io_context.stopped(); ++counter){
        // waits for watchdog request
        asio::read(client_socket,asio::buffer(allocated_memory_buffer),asio::transfer_exactly(sizeof(unsigned char)),ec);
        if(ec){
            std::printf("failed to send information\n terminating....\n");
            return 1;
        }

        allocated_memory_buffer[0] = 2;
        if(counter % 5 == 0){
            const auto local_copy = global_shared_gps_reading.load();
            copy_from_gps_reading_to_shared_memory(shared_memory->get_shared_memory_address(),local_copy);
            {
                std::lock_guard<std::mutex> g{camera_reading_mutex};
                copy_from_grayscale_image_1_to_shared_memory(shared_memory->get_shared_memory_address(),global_shared_camera_reading);
            }
            allocated_memory_buffer[1] = (unsigned char) 1; 
            allocated_memory_buffer[2] = (unsigned char) 1; 
        } else {
            auto local_copy = global_shared_gps_reading.load();
            copy_from_gps_reading_to_shared_memory(shared_memory->get_shared_memory_address(),local_copy);
            allocated_memory_buffer[1] = (unsigned char) 1; 
            allocated_memory_buffer[2] = (unsigned char) 0; 
        }
        
        size_t message_size = 2+1;
        asio::write(client_socket,asio::buffer(allocated_memory_buffer),asio::transfer_exactly(message_size),ec);
        if(ec){
            std::printf("failed to send information\n terminating....\n");
        }
    }
    }catch(...){
        std::cout << "failure was detected in either communication or shared memory operation\n";
    }
    camera_thread.join();
    gps_thread.join();
}