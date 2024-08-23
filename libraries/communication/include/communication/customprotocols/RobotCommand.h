#ifndef CURAN_ROBOTCOMMAND_HEADER_FILE_
#define CURAN_ROBOTCOMMAND_HEADER_FILE_

#include <array>
#include <stddef.h>
#include <stdint.h>
#include <cstring>

namespace curan{
namespace communication{

    struct RobotCommand{

        enum ControlCommand : size_t{
            FREEHAND,
            FIX_CURRENT_POSITION,
            NEEDLE_ALIGNED_CONFIGURATION
        };

        struct NeedleInformation{
            std::array<double,9> needle_aligment;
            std::array<double,3> needle_tip; 
        };

        RobotCommand() : needle_info{}, desired_command{FREEHAND}, buffer{} {

        }

        ~RobotCommand(){

        }
        
        NeedleInformation needle_info;
        ControlCommand desired_command;

        static constexpr size_t message_size = 9*sizeof(double)+3*sizeof(double)+sizeof(size_t);

        size_t body_size = message_size;

        std::array<unsigned char, message_size+sizeof(std::size_t)> buffer;

        inline std::size_t get_header_size(){
            return sizeof(body_size);
        }

        inline size_t get_body_size(){
            return body_size;
        }

        inline void deserialize_header(){
            std::memcpy(&body_size,buffer.data(),sizeof(size_t));
        }

        inline void deserialize(){
            size_t offset = sizeof(size_t);
            std::memcpy(needle_info.needle_aligment.data(),buffer.data()+offset,9*sizeof(double));
            offset += 9*sizeof(double);
            std::memcpy(needle_info.needle_tip.data(),buffer.data()+offset,3*sizeof(double));
            offset += 3*sizeof(double);
            std::memcpy(&desired_command,buffer.data()+offset,sizeof(size_t));
        }

        inline void serialize(){
            size_t offset = 0;
            std::memcpy(buffer.data()+offset,&body_size,sizeof(size_t));
            offset += sizeof(size_t);
            std::memcpy(buffer.data()+offset,needle_info.needle_aligment.data(),9*sizeof(double));
            offset += 9*sizeof(double);
            std::memcpy(buffer.data()+offset,needle_info.needle_tip.data(),3*sizeof(double));
            offset += 3*sizeof(double);
            std::memcpy(buffer.data()+offset,&desired_command,sizeof(size_t));
        }

        inline unsigned char* get_buffer(){
            return buffer.data();
        }

        inline unsigned char* get_body_buffer(){
            return buffer.data()+sizeof(size_t);
        }


    };

}
}

#endif