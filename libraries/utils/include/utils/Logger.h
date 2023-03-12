#ifndef CURAN_LOGGER_HEADER_FILE_
#define CURAN_LOGGER_HEADER_FILE_

#include <memory>
#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include <string>

namespace curan {
	namespace utils {

		class CustomLoger {

			enum Level{
				trace,
				debug,
				info
			};

			struct Content {
				std::string val;
				Level level;
			};

			std::list<Content> list_of_data;

			CustomLoger() {

			};

			void init_debug(std::string filename) {

			};
		};


		extern std::shared_ptr<spdlog::logger> console;
	}
}

#endif