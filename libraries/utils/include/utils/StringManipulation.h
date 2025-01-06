#ifndef CURAN_STRING_MANIPULATION_HEADER_FILE_
#define CURAN_STRING_MANIPULATION_HEADER_FILE_

#include <string>
#include <sstream>
#include <climits>

namespace curan
{
	namespace utilities
	{

		namespace internals
		{
			int countDigits(int n)
			{
				if(n<0)
					return countDigits(-n)+1;

				if (n == 0)
					return 1;

				int count = 0;
				while (n != 0){
					n = n / 10;
					++count;
				}
				return count;
			}

		}

		template <typename T>
		inline std::string to_string_with_precision(const T value, const int significant_digits = 6)
		{
			long long integer_part = value;
			char buffer[50];
			auto digits_length = internals::countDigits(integer_part);
			if(digits_length+1+significant_digits>=50)
				throw std::runtime_error("too many digits");
			auto result = std::snprintf(buffer, sizeof(buffer), "%*.*f",digits_length, significant_digits, value);
			if (result < 0)
				return std::string{};
			return std::string{buffer};
		}
	}
}

#endif