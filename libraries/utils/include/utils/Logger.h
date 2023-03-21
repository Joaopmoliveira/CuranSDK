#ifndef CURAN_LOGGER_MINE_HEADER_FILE_
#define CURAN_LOGGER_MINE_HEADER_FILE_

#include <list>
#include <string>
#include <sstream>
#include <fstream>
#include "SafeQueue.h"

namespace curan {
	namespace utils {

		constexpr int max_number_of_strings = 50;

		struct Logger {		

			Logger(const std::string& filename) {

			};

			~Logger() {

			};

			curan::utils::SafeQueue<std::string> outputqueue;
			std::list<std::string> previous_outputs;

			template<class U>
			friend Logger& operator<<(Logger& os, const U& L);
		};

		template<typename T>
		Logger& operator<< (Logger& io, const T& out) {
			std::stringstream s;
			s << out;
			io.outputqueue.push(s.str());
			return io;
		};


		extern Logger cout;
	}
}

#endif