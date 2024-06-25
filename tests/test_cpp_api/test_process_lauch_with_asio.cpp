#include <asio.hpp>
#include <boost/process.hpp>
#include <boost/process/async.hpp>
#include <iostream>

/*
    			child_process = std::make_unique<boost::process::child>(CURAN_BINARY_LOCATION"/VolumetricPathPlanning" CURAN_BINARY_SUFFIX, 
																		boost::process::std_out > child_out,
																		asio_ctx,
																		[&](int exit, const std::error_code& ec_in){
																			inbut->set_waiting_color(waiting_color_inactive);
																			child_process->terminate();
																			child_process = nullptr;
																		});
*/



int main(){
    boost::asio::io_context ios;
    std::error_code ec;
    boost::process::ipstream child_out;
    boost::process::child child("C:/Users/joaom/PlusApp-2.8.0.20191105-Win32/bin/PlusServer.exe",
                                "--config-file=C:/Dev/Curan/build/bin/resources/plus_config/wire_reconstructions/wire_reconstruction_recording/wire_reconstruction_recording/RecordingTest_config.xml",
                                "--verbose=4",
                                boost::process::std_out > child_out,
                                ec,
                                ios,
        boost::process::on_exit([&](int exit, const std::error_code& ec_in){
            std::cout << "failure PlusServer" << ec_in.message() << std::endl;
            ios.stop();
    }));

    ios.post([&](){
        
        while(!ios.stopped()){
            std::string ss;
            child_out >> ss;
            std::cout << ss << std::endl;
        }
    });

    ios.run();
    return 0;
}