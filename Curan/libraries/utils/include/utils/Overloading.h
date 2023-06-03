#ifndef CURAN_FLAG_HEADER_FILE_
#define CURAN_FLAG_HEADER_FILE_

namespace curan {
	namespace utilities {

		template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
		template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;
	}
}

#endif