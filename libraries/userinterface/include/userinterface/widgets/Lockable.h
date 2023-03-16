#ifndef CURAN_LOCKABLE_HEADER_FILE_
#define CURAN_LOCKABLE_HEADER_FILE_

#include <mutex>

namespace curan {
	namespace ui {
		template<typename Derived>
		class LockGuard;

		template<typename Derived>
		class Lockable {
			std::mutex mut;


			void lock() {
				mut.lock();
			}

			void unlock() {
				mut.unlock();
			}

			friend class LockGuard<Derived>;
		};

		template<typename Derived>
		class LockGuard {
			Lockable<Derived>* lock;
		public:
			LockGuard(Lockable<Derived>* supplied_lockable) : lock{ supplied_lockable } {
				lock->lock();
			}

			~LockGuard() {
				lock->unlock();
			}
		};
	}
}

#endif