#ifndef CURAN_STRING_MANIPULATION_HEADER_FILE_
#define CURAN_STRING_MANIPULATION_HEADER_FILE_

#include <string>
#include <sstream>

namespace curan {
	namespace utilities {
		template <typename T>
		inline std::string to_string_with_precision(const T value, const int significant_digits = 6)
		{
			char buffer[50];
			auto result = std::snprintf(buffer, sizeof(buffer), "%.*g", significant_digits, value);
			if(result < 0 )
				return std::string{};
			return std::string{buffer};
		}
	}
}

#endif