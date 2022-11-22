#include "asio.hpp"

namespace curan{
    namespace communication{
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
    }
}