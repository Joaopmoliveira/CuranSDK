#ifndef CkLibraryies_h_DEFINED
#define CkLibraryies_h_DEFINED

#include<vector>
#include <sigslot/signal.hpp>
#include "asio.hpp"
#include "DkUtilities.h"
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

namespace curan {
	/*
	The ck namespace is the namespace of the communication module inside curan.
	It contains all the functionallity related with communicating with external
	devices.
	*/
	namespace communication
	{
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
		class ProcessorOpenIGTLink {

		public:

			/*
			Enumeration of possible errors returned in the Ck module
			*/
			enum Status {
				CK_CLOSED = 0,
				CK_OPEN,
			};

			ProcessorOpenIGTLink(asio::io_context& io_context);

			~ProcessorOpenIGTLink();
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

		private:
			Status current_status = Status::CK_CLOSED;
			asio::io_context& io_context_;
			asio::ip::tcp::socket socket_;
			std::vector<unsigned char> body_data;
			igtl::MessageHeader::Pointer message_header;
			curan::utils::ThreadSafeQueue<igtl::MessageBase::Pointer> outbond_messages;
		};

		/*
		A Functor is an object which overrides the operator() that can
		jump start the io_context with a mock task to the execution context
		thus entering the cycle of the state machine implemented in the
		ProcessorOpenIGTLink class.
		*/
		class Functor {
			asio::io_context* io_context_;
			bool is_active = false;
		public:
			/*
			As soon as the constructor is called a dummy amount of work
			is given to the io_context to prevent if from returning until
			further async work is provided.
			*/
			Functor();

			void operator() ();

			bool is_running();
		};




		/*
		The channel manager contains channels that can be connected to
		different objects. These objects can submit themselfs in order
		to be notified.

		Each object which wishes to be warned should request a channel
		which is compatible with the language that they understand. To
		increase the maximum velocity that the application runs the
		burden of selecting the type of message is imposed on the listener.
		*/
		class ChannelManager
		{

		public:

			/*
			Enumeration of possible errors returned in the Ck module
			*/
			enum Error {
				SUCCESS = 0,
				CHANNEL_NOT_PRESENT,
				CHANNEL_ALREADY_PRESENT,
			};

			/*
			The error callback function should be submited by the user
			to be called at a latter point in time.
			*/
			using error_callback = std::function<void(Error)>;

			ChannelManager();
			static ChannelManager* Get();
			void start_channel(std::string name, asio::io_context& ctx, error_callback callback = error_callback());
			Error get_channel(std::string name, std::shared_ptr<ProcessorOpenIGTLink>& channel);
			Error terminate_channel(std::string name);
			void terminate_all_channels();

		private:
			std::mutex mut;
			Functor functor;
			std::map<std::string, std::shared_ptr<ProcessorOpenIGTLink>> openigtlink_channel_list;
		};
	}
}
#endif