#ifndef CURAN_CANCELABLE_HEADER_FILE_
#define CURAN_CANCELABLE_HEADER_FILE_

#include <memory>
/*
You can query and request for a connection to be
terminated at any moments notice.
*/
class Cancelable : std::enable_shared_from_this<Cancelable> {
	std::mutex mut;
	bool is_cancelled;

	Cancelable() : is_cancelled{ false } {}

public:

	[[nodiscard]] static std::shared_ptr<Cancelable> make_cancelable();

	void cancel();

	[[nodiscard]] bool operator() ();
};

#endif