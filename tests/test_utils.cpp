#include "utils\Flag.h"
#include "utils\Job.h"
#include "utils\Logger.h"
#include "utils\TheadPool.h"
#include "utils\ThreadSafeQueue.h"
#include <chrono>

int test_shared_flag() {
	auto flag = curan::utils::Flag::make_shared_flag();
	flag->clear();

	constexpr int number_of_min_miliseconds = 600;

	auto function1 = [flag, number_of_min_miliseconds]() {
		auto duration = std::chrono::milliseconds(number_of_min_miliseconds);
		std::this_thread::sleep_for(duration);
		flag->set();
	};

	auto function2 = [flag]() {
		curan::utils::console->info("started waiting for the flag");
		flag->wait();
		curan::utils::console->info("stopped waiting for the flag");
	};

	// we have two threads, one tells itself to sleep for a
	// given amount of time and the other waits until the
	// other has changed the common flag
	const std::chrono::time_point<std::chrono::system_clock> start =
		std::chrono::system_clock::now();
	std::thread d{ function2 };
	function1();
	d.join();
	const std::chrono::time_point<std::chrono::system_clock> end =
		std::chrono::system_clock::now();

	if (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() <= number_of_min_miliseconds)
		curan::utils::console->error("the total amount of time waited for the flag was smaller than what was expected");
	else 
		curan::utils::console->info("the obtained result is in accordance with what was expected");

	return 0;
}

int main() {

	return 0;
}


void test_job_and_thread_pool() {

}

void test_logger() {

}

void test_thread_safe_queue() {

}