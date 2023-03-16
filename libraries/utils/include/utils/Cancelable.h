#ifndef CURAN_CANCELABLE_HEADER_FILE_
#define CURAN_CANCELABLE_HEADER_FILE_

#include <mutex>
#include <vector>

namespace curan {
	namespace utils {

		class Cancelable : std::enable_shared_from_this<Cancelable> {
			std::mutex mut;
			bool is_cancelled;

			Cancelable() : is_cancelled{ false } {}

		public:

			[[nodiscard]] static std::shared_ptr<Cancelable> make_cancelable();

			void cancel();

			[[nodiscard]] bool operator() ();
		};

		template<typename Derived>
		class Connectable {
			std::vector<std::shared_ptr<Cancelable>> cancelable;
		public:
			void submit_cancelable(std::shared_ptr<Cancelable> submited_cancelable) {
				cancelable.push_back(submited_cancelable);
			}

			~Connectable() {
				for (auto& canc : cancelable)
					if(canc)
						canc->cancel();
			}
		};

	}
}
#endif