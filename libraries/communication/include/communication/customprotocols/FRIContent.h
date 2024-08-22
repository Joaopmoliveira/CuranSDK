#ifndef CURAN_FRIMESSAGE_HEADER_FILE_
#define CURAN_FRIMESSAGE_HEADER_FILE_

#include <array>
#include <stddef.h>
#include <stdint.h>
#include <cstring>

namespace curan{
namespace communication{

    struct FRIMessage{

        FRIMessage() : measured_torques{}, external_torques{}, angles{}, buffer{} {

        }

        ~FRIMessage(){

        }

        static constexpr size_t n_joints = 7;
        
        std::array<double,n_joints> angles;
        std::array<double,n_joints> external_torques;
        std::array<double,n_joints> measured_torques;

        static constexpr size_t message_size = 3*n_joints*sizeof(double);

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
            std::memcpy(angles.data(),buffer.data()+offset,n_joints*sizeof(double));
            offset += n_joints*sizeof(double);
            std::memcpy(external_torques.data(),buffer.data()+offset,n_joints*sizeof(double));
            offset += n_joints*sizeof(double);
            std::memcpy(measured_torques.data(),buffer.data()+offset,n_joints*sizeof(double));
        }

        inline void serialize(){
            size_t offset = 0;
            std::memcpy(buffer.data()+offset,&body_size,sizeof(size_t));
            offset += sizeof(size_t);
            std::memcpy(buffer.data()+offset,angles.data(),n_joints*sizeof(double));
            offset += n_joints*sizeof(double);
            std::memcpy(buffer.data()+offset,external_torques.data(),n_joints*sizeof(double));
            offset += n_joints*sizeof(double);
            std::memcpy(buffer.data()+offset,measured_torques.data(),n_joints*sizeof(double));
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