#ifndef CURAN_PROCESS_HANDLER_HEADER_FILE_
#define CURAN_PROCESS_HANDLER_HEADER_FILE_

#include <array>

namespace curan{
namespace communication{

    struct ProcessHandler{

        enum Signals : uint32_t{
            SHUTDOWN_SAFELY,
            HEART_BEAT,
            ACKNOLEGE_HEAR_BEAT,
            SIGNALS_SIZE_
        };

        ProcessHandler(){

        }

        explicit ProcessHandler(const Signals& sig) : signal_to_process{sig}{

        }

        ~ProcessHandler(){

        }
        
        Signals signal_to_process;

        static constexpr size_t message_size = sizeof(Signals);

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
            std::memcpy(&signal_to_process,buffer.data()+offset,message_size);
        }

        inline void serialize(){
            size_t offset = 0;
            std::memcpy(buffer.data()+offset,&body_size,sizeof(body_size));
            offset += sizeof(body_size);
            std::memcpy(buffer.data()+offset,&signal_to_process,message_size);
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