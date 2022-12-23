#ifndef CURAN_PROCESSOROIGTL_HEADER_FILE_
#define CURAN_PROCESSOROIGTL_HEADER_FILE_

#include "asio.hpp"
#include <asio/awaitable.hpp>
#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlTransformMessage.h"
#include "igtlPositionMessage.h"
#include "igtlImageMessage.h"
#include "igtlClientSocket.h"
#include "igtlStatusMessage.h"

#if OpenIGTLink_PROTOCOL_VERSION >= 2
#include "igtlSensorMessage.h"
#include "igtlPointMessage.h"
#include "igtlTrajectoryMessage.h"
#include "igtlStringMessage.h"
#include "igtlTrackingDataMessage.h"
#include "igtlQuaternionTrackingDataMessage.h"
#include "igtlCapabilityMessage.h"
#endif

#include "utils/ThreadSafeQueue.h"
#include <sigslot/signal.hpp>

namespace curan{
    namespace communication{
		/*
		Given that the underlying library used to communicate with external applications is ASIO we 
		need a single executor, i.e. io_context, which records tasks to be performed at a latter point 
		in time. This function initializes a static io_context which lasts throught the duration of the
		program. When the developer wants to use it, he/she can request a pointer to this static variable
		as such:

		asio::io_context* context;
		curan::ck::GetIOContext(&context);
		// The pointer is now available and ready to be used.

		*/
		void GetIOContext(asio::io_context** io_context);

        /*
		This class is prepared to be used with a asio io_context which is only called from a single thread.
		If more threads call the io_context the queue which contains outstanding work does not have any
		protection against concurrent acess.
		This could be solved with a mutex, but for the moment it was decided that the work is serialized
		and no concurrent acess may happen. This means that a single io_context can be executed at once.
		*/
		class ProcessorOIGTL {

		public:

			/*
			Enumeration of possible errors returned in the Ck module
			*/
			enum Status {
				CK_CLOSED = 0,
				CK_OPEN,
			};

			ProcessorOIGTL(asio::io_context& io_context);

			~ProcessorOIGTL();
			void write(igtl::MessageBase::Pointer msg);
			void close();
			void connect(const asio::ip::tcp::resolver::results_type& endpoints);
			Status getStatus();
			sigslot::signal<igtl::MessageBase::Pointer> signal_received;
			sigslot::signal<Status> signal_connection_status;

		private:

			void change_status(Status status);
			void handle_connect(const asio::error_code& error);
			void handle_read_header(const asio::error_code& error);
			void handle_read_body(const asio::error_code& error);
			void do_write(igtl::MessageBase::Pointer msg);
			void handle_write(const asio::error_code& error);
			void do_close();

			Status current_status = Status::CK_CLOSED;
			asio::io_context& io_context_;
			asio::ip::tcp::socket socket_;
			std::vector<unsigned char> body_data;
			std::mutex writing_mutex_block;
			igtl::MessageHeader::Pointer message_header;
			curan::utils::ThreadSafeQueue<igtl::MessageBase::Pointer> outbond_messages;
		};
    }
}

#endif