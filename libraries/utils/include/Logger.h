#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/sinks/stdout_color_sinks.h"

namespace curan {
	namespace utils {
		extern std:shared_ptr<spdlog::logger> console;
	}
}