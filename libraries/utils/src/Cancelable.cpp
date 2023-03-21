#include "utils/Cancelable.h"

#include <mutex>

namespace curan {
namespace utils {
std::shared_ptr<Cancelable> Cancelable::make_cancelable() {
	return std::shared_ptr<Cancelable>(new Cancelable());
}

void Cancelable::cancel() {
	std::lock_guard<std::mutex> g(mut);
	is_cancelled = 0;
}

bool Cancelable::operator() () {
	std::lock_guard<std::mutex> g(mut);
	return is_cancelled;
}
}
}