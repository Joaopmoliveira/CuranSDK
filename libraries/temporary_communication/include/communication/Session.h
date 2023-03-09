#ifndef CURAN_SESSION_HEADER_FILE_
#define CURAN_SESSION_HEADER_FILE_

namespace curan {
    namespace io {
        /*
        Now we define the custom message handlers that each protocol will implement.
        When we wish to
        */
        class Session {
            friend generic_protocol_requirments;

            virtual void start(message_request&& get_message) = 0;
            virtual void broadcast(error_communication communication, std::shared_ptr<curan::utils::memory_buffer> message) = 0;
            virtual void post(std::shared_ptr<curan::utils::memory_buffer> message) = 0;

            static std::shared_ptr<Session> make_session();

            static std::shared_ptr<Session> make_session();
        };
    }
}

#endif