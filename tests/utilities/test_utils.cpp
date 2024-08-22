#include "utils/Semaphore.h"
#include "utils/Job.h"
#include "utils/Logger.h"
#include "utils/TheadPool.h"
#include "utils/SafeQueue.h"
#include "utils/MemoryUtils.h"
#include <chrono>
#include <iostream>

/**
The utilities library is at the core of curan. All other modules of the 
medical viewer use the constructs provided by this module. Lets go into the 
details!
*/

int test_shared_flag(std::shared_ptr<curan::utilities::ThreadPool> shared_pool) {
	curan::utilities::Semaphore flag;
	constexpr int number_of_min_miliseconds = 600;

	auto function1 = [&flag, number_of_min_miliseconds]() {
		auto duration = std::chrono::milliseconds(number_of_min_miliseconds);
		std::this_thread::sleep_for(duration);
		flag.trig();
	};

	auto function2 = [&flag]() {
		std::cout << "started waiting for the flag\n";
		flag.wait();
		std::cout << "stopped waiting for the flag\n";
	};

	const std::chrono::time_point<std::chrono::system_clock> start =
		std::chrono::system_clock::now();
	std::thread d{ function2 };
	function1();
	d.join();
	const std::chrono::time_point<std::chrono::system_clock> end =
		std::chrono::system_clock::now();

	if (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() <= number_of_min_miliseconds)
		std::cout << "the total amount of time waited for the flag was smaller than what was expected\n";
	else 
		std::cout << "the obtained result is in accordance with what was expected\n";

	return 0;
}

int test_job_and_thread_pool(std::shared_ptr<curan::utilities::ThreadPool> shared_pool) {
	using namespace curan::utilities;
	curan::utilities::Semaphore flag1;

	curan::utilities::Semaphore flag2;

	curan::utilities::Job job1{"This is a test to make sure that the thread pool works",[&]() {
		flag1.wait();
	}};

	curan::utilities::Job job2{"This is a test to make sure that the thread pool works",[&]() {
		flag1.wait();
	}};

	curan::utilities::Job job3{"This is a test to make sure that the thread pool works",[&]() {
		flag2.wait();
	}};

	curan::utilities::Job job4{"This is a test to make sure that the thread pool works",[&]() {
		flag2.wait();
	}};

	//expected behavior once we submit the first job we expect the number of tasks to increment by 1, then 2 ...
	int number_of_tasks = 0;
	int number_of_tasks_in_queue = 0;
	shared_pool->get_number_tasks(number_of_tasks, number_of_tasks_in_queue);
	std::string message = "Number of tasks (pending + execution): (" +std::to_string(number_of_tasks) +" + "+ std::to_string(number_of_tasks_in_queue) + ") (expected 0) \n";
	std::cout << message;
	shared_pool->submit(job1);
	shared_pool->get_number_tasks(number_of_tasks, number_of_tasks_in_queue);
	message = "Number of tasks (pending + execution): (" + std::to_string(number_of_tasks) + " + " + std::to_string(number_of_tasks_in_queue) + ") (expected 1) \n";
	std::cout << message;
	shared_pool->submit(job2);
	shared_pool->get_number_tasks(number_of_tasks, number_of_tasks_in_queue);
	message = "Number of tasks (pending + execution): (" + std::to_string(number_of_tasks) + " + " + std::to_string(number_of_tasks_in_queue) + ") (expected 2) \n";
	std::cout << message;
	shared_pool->submit(job3);
	shared_pool->submit(job4);
	shared_pool->get_number_tasks(number_of_tasks, number_of_tasks_in_queue);
	message = "Number of tasks (pending + execution): (" + std::to_string(number_of_tasks) + " + " + std::to_string(number_of_tasks_in_queue) + ") (expected 4) \n";
	std::cout << message;
	flag1.trig();
	shared_pool->get_number_tasks(number_of_tasks, number_of_tasks_in_queue);
	message = "Number of tasks (pending + execution): (" + std::to_string(number_of_tasks) + " + " + std::to_string(number_of_tasks_in_queue) + ") (expected <4)\n ";
	std::cout << message;
	flag2.trig();
	shared_pool->get_number_tasks(number_of_tasks, number_of_tasks_in_queue);
	message = "Number of tasks (pending + execution): (" + std::to_string(number_of_tasks) + " + " + std::to_string(number_of_tasks_in_queue) + ") (expected <4) \n";
	std::cout << message;
	std::cout << message;
	return 0;
}

struct test_target {
	int val;

	test_target() : val{ 0 } {};
};

void test_thread_safe_queue(std::shared_ptr<curan::utilities::ThreadPool> shared_pool) {
	curan::utilities::SafeQueue<test_target> safe_queue;
	auto put_in = [&safe_queue](test_target pop) {
		safe_queue.push(pop);
	};
	auto function_to_execute = [&safe_queue]() {
		try {
			std::cout << "starting to wait for popable\n";
			while (!safe_queue.is_invalid()) {
				if(auto test = safe_queue.wait_and_pop(); test)
					std::cout << "Received a poopable! yeye = " << (*test).val << "\n";
			}
			std::cout << "finished to wait for popable\n";
		}
		catch (...) {
			std::cout << "something very wrong happened\n";
		}
	};
	

	std::thread thread_to_run{ function_to_execute };
	int number_of_min_miliseconds = 10;
	try {	
		std::vector<int> values_to_test = {1,2,3,4,5,6,7,8,9,10};
		for (auto val : values_to_test) {
			test_target pop;
			pop.val = val;
			put_in(pop);
			auto duration = std::chrono::milliseconds(number_of_min_miliseconds);
			std::this_thread::sleep_for(duration);
		}
		std::cout << "setting boolean variable to false\n";
		safe_queue.invalidate();
	} catch (...) {
		std::cout << "something very wrong in the main thread\n";
	}
	thread_to_run.join();
}

void test_memory_buffers() {
	std::shared_ptr<curan::utilities::MemoryBuffer> buff_of_interest;
	{
		std::string value_to_control = "1_2_3_4_5_6_7_8_9_10_11";
		std::cout << "Expected buffer is: " << value_to_control << "\n";
		buff_of_interest = curan::utilities::CopyBuffer::make_shared(value_to_control.data(), value_to_control.size());
		//value to constrol is deleted but the memory has already been copied into the memory_buffer class
	}
	
	std::cout << "First buffer is: " << buff_of_interest << "\n";
	
	{
		std::string value_to_control = "1_2_3_4_5_6_7_8_9_10_12";
		std::cout << "Expected buffer is: " << value_to_control << "\n";
		std::shared_ptr<std::string> shared_memory_to_control = std::shared_ptr<std::string>(new std::string(value_to_control));

		buff_of_interest = curan::utilities::CaptureBuffer::make_shared(shared_memory_to_control->data(), shared_memory_to_control->size(),shared_memory_to_control);
		//value to constrol is deleted as well as the local copy of the shared_memory_control, but the shared_pointer is
		//stored into the lamda which is then copied into the capture memory buffer
	}
	
	std::cout << "Second buffer is: " << buff_of_interest << "\n";
}

int main() {
	//initualize the thread pool;
	auto projeto = curan::utilities::ThreadPool::create(10);

	test_shared_flag(projeto);
	test_job_and_thread_pool(projeto);
	test_thread_safe_queue(projeto);
	test_memory_buffers();
	return 0;
}

