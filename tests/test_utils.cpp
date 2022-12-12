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

int test_job_and_thread_pool() {
	auto flag1 = curan::utils::Flag::make_shared_flag();
	flag1->clear();
	auto flag2 = curan::utils::Flag::make_shared_flag();
	flag2->clear();
	curan::utils::ThreadPool* pool = curan::utils::ThreadPool::Get();
	curan::utils::Job job1;
	job1.description = "This is a test to make sure that the thread pool works";
	job1.function_to_execute = [flag1]() {
		flag1->wait();
	};
	curan::utils::Job job2;
	job2.description = "This is a test to make sure that the thread pool works";
	job2.function_to_execute = [flag1]() {
		flag1->wait();
	};
	curan::utils::Job job3;
	job3.description = "This is a test to make sure that the thread pool works";
	job3.function_to_execute = [flag2]() {
		flag2->wait();
	};
	curan::utils::Job job4;
	job4.description = "This is a test to make sure that the thread pool works";
	job4.function_to_execute = [flag2]() {
		flag2->wait();
	};

	//expected behavior once we submit the first job we expect the number of tasks to increment by 1, then 2 ...
	int number_of_tasks = 0;
	int number_of_tasks_in_queue = 0;
	pool->GetNumberTasks(number_of_tasks, number_of_tasks_in_queue);
	std::string message = "Number of tasks (pending + execution): (" +std::to_string(number_of_tasks) +" + "+ std::to_string(number_of_tasks_in_queue) + ") (expected 0) ";
	curan::utils::console->info(message);
	pool->Submit(job1);
	pool->GetNumberTasks(number_of_tasks, number_of_tasks_in_queue);
	message = "Number of tasks (pending + execution): (" + std::to_string(number_of_tasks) + " + " + std::to_string(number_of_tasks_in_queue) + ") (expected 1) ";
	curan::utils::console->info(message);
	pool->Submit(job2);
	pool->GetNumberTasks(number_of_tasks, number_of_tasks_in_queue);
	message = "Number of tasks (pending + execution): (" + std::to_string(number_of_tasks) + " + " + std::to_string(number_of_tasks_in_queue) + ") (expected 2) ";
	curan::utils::console->info(message);
	pool->Submit(job3);
	pool->Submit(job4);
	pool->GetNumberTasks(number_of_tasks, number_of_tasks_in_queue);
	message = "Number of tasks (pending + execution): (" + std::to_string(number_of_tasks) + " + " + std::to_string(number_of_tasks_in_queue) + ") (expected 4) ";
	curan::utils::console->info(message);
	flag1->set();
	pool->GetNumberTasks(number_of_tasks, number_of_tasks_in_queue);
	message = "Number of tasks (pending + execution): (" + std::to_string(number_of_tasks) + " + " + std::to_string(number_of_tasks_in_queue) + ") (expected <4) ";
	curan::utils::console->info(message);
	flag2->set();
	pool->GetNumberTasks(number_of_tasks, number_of_tasks_in_queue);
	message = "Number of tasks (pending + execution): (" + std::to_string(number_of_tasks) + " + " + std::to_string(number_of_tasks_in_queue) + ") (expected <4) ";
	curan::utils::console->info(message);
	curan::utils::console->info("the thread pool has been stopped");
	return 0;
}

struct PoPable {
	int val;

	PoPable() : val{ 0 } {
	
	};


};

void test_thread_safe_queue() {
	PoPable pop;
	curan::utils::ThreadSafeQueue<PoPable> safe_queue;
	auto put_in = [&safe_queue](PoPable pop) {
		safe_queue.push(pop);
	};
	auto function_to_execute = [&safe_queue]() {
		try {
			PoPable temp;
			curan::utils::console->info("starting to wait for popable");
			while (!safe_queue.is_invalid()) {
				safe_queue.wait_and_pop(temp);
				curan::utils::console->info("Received a poopable! yeye = {}", temp.val);
			}
			curan::utils::console->info("finished to wait for popable");
		}
		catch (...) {
			curan::utils::console->error("something very wrong happened");
		}
	};
	

	std::thread thread_to_run{ function_to_execute };
	int number_of_min_miliseconds = 10;
	try {	
		std::vector<int> values_to_test = {1,2,3,4,5,6,7,8,9,10};
		for (auto val : values_to_test) {
			pop.val = val;
			put_in(pop);
			auto duration = std::chrono::milliseconds(number_of_min_miliseconds);
			std::this_thread::sleep_for(duration);
		}
		curan::utils::console->info("setting boolean variable to false");
		safe_queue.invalidate();
	} catch (...) {
		curan::utils::console->error("something very wrong in the main thread");
	}
	thread_to_run.join();
}

//TODO the thread pool has a bug which is annoying...

int main() {
	test_shared_flag();
	test_job_and_thread_pool();
	test_thread_safe_queue();
	return 0;
}

