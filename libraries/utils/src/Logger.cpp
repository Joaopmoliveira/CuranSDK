#include "utils/Logger.h"

namespace curan{
    namespace utils{
       		std::shared_ptr<spdlog::logger>console = spdlog::stdout_color_mt("logger"); 
    }
}