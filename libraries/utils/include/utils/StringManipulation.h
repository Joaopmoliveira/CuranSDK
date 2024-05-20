#ifndef CURAN_STRING_MANIPULATION_HEADER_FILE_
#define CURAN_STRING_MANIPULATION_HEADER_FILE_

#include <string>
#include <sstream>

namespace curan {
	namespace utilities {
		template <typename T>
		std::string to_string_with_precision(const T a_value, const int n = 6)
		{
			std::ostringstream out;
			out.precision(n);
			out << std::fixed << a_value;
			return out.str();
		}
	}
}

#endif