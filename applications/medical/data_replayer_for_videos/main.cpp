#include <iostream>
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"
#include <iostream>
#include <thread>
#include "utils/Logger.h"
#include "utils/Flag.h"
#include "utils/Reader.h"
#include "utils/TheadPool.h"
#include <csignal>
#include <fstream>
#include <iostream>
#include <boost/process.hpp>
#include <boost/asio/read_until.hpp>
#include <nlohmann/json.hpp>

asio::io_context context;
boost::asio::io_context in_asio_ctx;

void signal_handler(int signal)
{
	context.stop();
    in_asio_ctx.stop();
}

void start_joint_tracking(std::shared_ptr<curan::communication::Server<curan::communication::protocols::fri>> server, const std::string& path)
{
	asio::io_context &in_context = server->get_context();
	
	int val = 40;
	std::string name = "Base";
    
    std::ifstream input_data{path};
    if(!input_data.is_open()){
        std::cout << "failed to read joint tracking data: " << path << std::endl;
        signal_handler(1);
        return;
    }

    std::stringstream ss;
    ss << input_data.rdbuf();
    auto joint_readings = curan::utilities::convert_matrix(ss,',');

    if(joint_readings.cols()!=7 || joint_readings.rows()<3)
        throw std::runtime_error("the dimensions do not match what is expected");
    
    size_t i = 0;

	while (!in_context.stopped() && i<joint_readings.rows())
	{
		const auto start = std::chrono::high_resolution_clock::now();
		std::shared_ptr<curan::communication::FRIMessage> message = std::shared_ptr<curan::communication::FRIMessage>(new curan::communication::FRIMessage());

        Eigen::Matrix<double,7,1> joint_at_index = joint_readings.block<1,7>(i,0);
        for(size_t j = 0; j< 7; ++j)
            message->angles[j] = joint_at_index[j];
		message->serialize();
		auto to_send = curan::utilities::CaptureBuffer::make_shared(message->get_buffer(), message->get_body_size() + message->get_header_size(),message);
		server->write(to_send);
		const auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000.0 / val)) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
        ++i;
        if(i>=joint_readings.rows())
            i = 0;
	}
}

int intermediate(int argc, char* argv[]){
    if(argc != 2){
        std::cout << "to call this executable, you need to provide two arguments:\n"
                  << "DataReplayer joint_recording.txt plus_configuration_file.xml";
        return 1;
    }

    std::ifstream json_file_in(argv[1]);
    if (!json_file_in.is_open())
    {
        std::cout << "failure to read the specification file" << std::endl;
        return 1;
    }
    
    unsigned short port_fri = 50010;
    std::shared_ptr<curan::communication::Server<curan::communication::protocols::fri>> server_joints;


	boost::asio::streambuf plus_buf;
	std::error_code plus_ec;
	boost::process::async_pipe plus_out{in_asio_ctx};

    auto pool = curan::utilities::ThreadPool::create(3);

    std::unique_ptr<boost::process::child> plus_process;

	std::signal(SIGINT, signal_handler);

    nlohmann::json needle_calibration_specification;
    json_file_in >> needle_calibration_specification;

    if (needle_calibration_specification.contains("plus_configuration_file"))
    {
        std::string path_to_plus = needle_calibration_specification["plus_configuration_file"];
	    plus_process = std::make_unique<boost::process::child>(CURAN_PLUS_EXECUTABLE_PATH, std::string{"--config-file="} + path_to_plus,"--verbose=1",
															   boost::process::std_out > plus_out,
															   plus_ec,
															   in_asio_ctx,
															   boost::process::on_exit([](int exit, const std::error_code &ec_in)
																					   {
																						   if (exit)
																							   std::cout << "Plus failure" << ec_in.message() << std::endl;
																					   }));
        struct Async{
            boost::asio::streambuf& plus_buf;
	        std::error_code& plus_ec;
	        boost::process::async_pipe& plus_out;

            Async(boost::asio::streambuf& in_plus_buf,std::error_code& in_plus_ec,boost::process::async_pipe& in_plus_out) : plus_buf{in_plus_buf},plus_ec{in_plus_ec},plus_out{in_plus_out}{

            };

            void operator() (const boost::system::error_code &ec, std::size_t size){
			    static std::string line;
			    static std::istream istr(&plus_buf);
			    if (!ec){
				    std::getline(istr, line);
				    std::cout << "plus >> " << line << std::endl;
			    }
			    if (!ec)
                    post_async();
            };

            void post_async(){
                boost::asio::async_read_until(plus_out,plus_buf,'\n',*this);
            }

        };

        Async post_reading_operations{plus_buf,plus_ec,plus_out};
        post_reading_operations.post_async();
		
        pool->submit(curan::utilities::Job{"run process tracking",[&](){ std::cout << "running plus"; in_asio_ctx.run();}});
    } else {
        std::cout << "no plus configuration file" << std::endl;
    }

    if (needle_calibration_specification.contains("joint_configuration_file"))
    {
        std::string  path_to_joints = needle_calibration_specification["joint_configuration_file"];
        server_joints = curan::communication::Server<curan::communication::protocols::fri>::make(context, port_fri);
        pool->submit(curan::utilities::Job{"run joint tracking",[&](){start_joint_tracking(server_joints, path_to_joints);}});
    }else {
        std::cout << "no joint configuration file" << std::endl;
    }
    auto work_protection = asio::make_work_guard(context);
	context.run();
    if (plus_process)
		plus_process->terminate();
	plus_process = nullptr;
	plus_out.async_close();
    std::cout << "terminating" << std::endl;
    return 0;
}

int main(int argc, char* argv[]){
    try{
        return intermediate(argc,argv);
    } catch(std::runtime_error& e){
        std::cout << "exception: " << e.what() << std::endl; 
    } catch(...){
        std::cout << "unknown exception"<< std::endl; 
    }
    return 1;
}