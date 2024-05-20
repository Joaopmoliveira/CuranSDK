#ifndef CURAN_LOGGER_MINE_HEADER_FILE_
#define CURAN_LOGGER_MINE_HEADER_FILE_

#include <list>
#include <string>
#include <sstream>
#include <fstream>
#include "SafeQueue.h"
#include <iostream>

namespace curan {
	namespace utilities {

		constexpr int max_number_of_strings = 50;

		

		struct Logger {		

			Logger(const std::string& filename) {

			};

			~Logger() {

			};

			SafeQueue<std::string> outputqueue;

			template<class U>
			friend Logger& operator<<(Logger& os, const U& L);
		};

		template<typename T>
		Logger& operator<< (Logger& io, const T& out) {
			std::stringstream s;
			s << out;
			std::cout << s.str();
			if(io.outputqueue.size()<max_number_of_strings)
				io.outputqueue.push(s.str());
			return io;
		};


		extern Logger cout;
	}
}

#endif