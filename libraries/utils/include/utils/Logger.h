#ifndef CURAN_LOGGER_HEADER_FILE_
#define CURAN_LOGGER_HEADER_FILE_

#include <memory>
#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/sinks/stdout_color_sinks.h"

namespace curan {
	namespace utils {
		extern std::shared_ptr<spdlog::logger> console;
	}
}

#endif