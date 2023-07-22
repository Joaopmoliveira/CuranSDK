#ifndef CURAN_FRIMESSAGE_HEADER_FILE_
#define CURAN_FRIMESSAGE_HEADER_FILE_

#include <array>

namespace curan{
namespace communication{

    struct FRIMessage{
        static constexpr size_t NUMBER_OF_JOINTS = 7;
        
        std::array<double,NUMBER_OF_JOINTS> angles;
        std::array<double,NUMBER_OF_JOINTS> external_torques;
        std::array<double,NUMBER_OF_JOINTS> measured_torques;

        static constexpr size_t message_size = 3*NUMBER_OF_JOINTS*sizeof(double);

        size_t body_size = message_size;

        std::array<unsigned char, message_size+sizeof(size_t)> buffer;

        inline size_t get_header_size(){
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
            std::memcpy(angles.data(),buffer.data()+offset,NUMBER_OF_JOINTS*sizeof(double));
            offset += NUMBER_OF_JOINTS*sizeof(double);
            std::memcpy(external_torques.data(),buffer.data()+offset,NUMBER_OF_JOINTS*sizeof(double));
            offset += NUMBER_OF_JOINTS*sizeof(double);
            std::memcpy(measured_torques.data(),buffer.data()+offset,NUMBER_OF_JOINTS*sizeof(double));
        }

        inline void serialize(){
            size_t offset = 0;
            std::memcpy(buffer.data()+offset,&body_size,NUMBER_OF_JOINTS*sizeof(double));
            offset += sizeof(size_t);
            std::memcpy(buffer.data()+offset,angles.data(),NUMBER_OF_JOINTS*sizeof(double));
            offset += NUMBER_OF_JOINTS*sizeof(double);
            std::memcpy(buffer.data()+offset,external_torques.data(),NUMBER_OF_JOINTS*sizeof(double));
            offset += NUMBER_OF_JOINTS*sizeof(double);
            std::memcpy(buffer.data()+offset,measured_torques.data(),NUMBER_OF_JOINTS*sizeof(double));
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