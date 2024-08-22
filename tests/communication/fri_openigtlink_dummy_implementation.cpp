#include <thread>
#include <atomic>
#include <csignal>
#include <asio.hpp>
#include "communication/ProtoFRI.h"
#include "communication/Server.h"
#include "communication/Client.h"
#include "utils/Logger.h"

unsigned short port = 50000;
asio::io_context io_context;

void signal_handler(int signal){
    io_context.stop();
}

int server_function(){
    try{
	auto server_joints = curan::communication::Server<curan::communication::protocols::fri>::make( io_context ,port );
    double counter = 0.0;
    while(!io_context.stopped()){
        std::shared_ptr<curan::communication::FRIMessage> message = std::make_shared<curan::communication::FRIMessage>();
        for(size_t link = 0 ; link < curan::communication::FRIMessage::n_joints ; ++link){
		    message->angles[link] = counter;
		    message->external_torques[link] = counter;
		    message->measured_torques[link] = counter; 
        }

		message->serialize();

		auto to_send = curan::utilities::CaptureBuffer::make_shared(message->get_buffer(),message->get_body_size()+message->get_header_size(),message);
		server_joints->write(to_send);

        counter += 1.000;   
        std::this_thread::sleep_for(std::chrono::milliseconds(30));    
    }
    return 0;
    } catch(...){
        std::cout << "exception thrown\n";
        return 0;
    }
}

int client_callback(const size_t& loc, const std::error_code& err, std::shared_ptr<curan::communication::FRIMessage> message){
    std::printf("hi mark\n");
    if (!err) {
        for(size_t link = 0 ; link< curan::communication::FRIMessage::n_joints ; ++link){
            std::printf("values: %f %f %f \n",message->angles[link],message->external_torques[link],message->measured_torques[link]);
        }
    }else {
        std::printf("error\n");
    }
    return 0;
}

void utilities_parser(){
    while(!io_context.stopped()){
        if(auto previous_string = curan::utilities::cout.outputqueue.wait_and_pop(); previous_string)
            std::cout << *previous_string << "\n";
    }

}

int main(){
    try{
    std::signal(SIGINT, signal_handler);
    std::thread server_thread{server_function};
    std::thread parser{utilities_parser};
    asio::ip::tcp::resolver resolver(io_context);
	auto client_joints = curan::communication::Client<curan::communication::protocols::fri>::make( io_context ,resolver.resolve("localhost", std::to_string(port)));
    client_joints->connect(client_callback);

    std::shared_ptr<curan::communication::FRIMessage> message = std::make_shared<curan::communication::FRIMessage>();
    for(size_t link = 0 ; link < curan::communication::FRIMessage::n_joints ; ++link){
		message->angles[link] = 10;
	    message->external_torques[link] = 10;
	    message->measured_torques[link] = 10; 
    }

	message->serialize();

	auto to_send = curan::utilities::CaptureBuffer::make_shared(message->get_buffer(),message->get_body_size()+message->get_header_size(),message);

    client_joints->write(to_send);
    
    io_context.run();
    server_thread.join();
    parser.join();
    std::cout << "exiting\n";
    return 0;
    }
    catch(...){
        std::cout << "exception main thrown\n";
    }
}