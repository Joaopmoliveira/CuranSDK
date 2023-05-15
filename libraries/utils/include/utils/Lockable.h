#ifndef CURAN_LOCKABLE_HEADER_FILE_
#define CURAN_LOCKABLE_HEADER_FILE_

#include <mutex>

namespace curan {
	namespace utils {

		template<typename Derived>
		class Lockable {
			std::mutex mut;

		public:
			std::mutex& get_mutex() {
				return mut;
			}
		};
	}
}

#endif