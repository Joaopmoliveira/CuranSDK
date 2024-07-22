#ifndef CURAN_PROTOIGTL_HEADER_FILE_
#define CURAN_PROTOIGTL_HEADER_FILE_

#include <string>
#include <asio.hpp>

namespace curan {
	namespace communication {

		class SerialClient{
            struct Info {
                char termination_character = 'e';
                asio::io_service& service;
                uint32_t baud_rate = 9600;
                std::string port_name;
            };

            asio::serial_port serial;

            SerialClient(Info& info);

            std::string read(size_t n);
            void write(const std::string& s);
            void close();
        };
	}
}


#endif 