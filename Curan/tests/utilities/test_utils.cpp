#include "utils\Flag.h"
#include "utils\Job.h"
#include "utils\Logger.h"
#include "utils\TheadPool.h"
#include "utils\SafeQueue.h"
#include "utils\MemoryUtils.h"
#include <chrono>
#include <iostream>

/**
The utilities library is at the core of curan. All other modules of the 
medical viewer use the constructs provided by this module. Lets go into the 
details!
*/

int test_shared_flag() {
	auto flag = curan::utilities::Flag::make_shared_flag();
	flag->clear();

	constexpr int number_of_min_miliseconds = 600;

	auto function1 = [flag, number_of_min_miliseconds]() {
		auto duration = std::chrono::milliseconds(number_of_min_miliseconds);
		std::this_thread::sleep_for(duration);
		flag->set();
	};

	auto function2 = [flag]() {
		std::cout << "started waiting for the flag\n";
		flag->wait();
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

int test_job_and_thread_pool() {
	using namespace curan::utilities;
	auto flag1 = curan::utilities::Flag::make_shared_flag();
	flag1->clear();
	auto flag2 = curan::utilities::Flag::make_shared_flag();
	flag2->clear();

	curan::utilities::Job job1;
	job1.description = "This is a test to make sure that the thread pool works";
	job1.function_to_execute = [flag1]() {
		flag1->wait();
	};
	curan::utilities::Job job2;
	job2.description = "This is a test to make sure that the thread pool works";
	job2.function_to_execute = [flag1]() {
		flag1->wait();
	};
	curan::utilities::Job job3;
	job3.description = "This is a test to make sure that the thread pool works";
	job3.function_to_execute = [flag2]() {
		flag2->wait();
	};
	curan::utilities::Job job4;
	job4.description = "This is a test to make sure that the thread pool works";
	job4.function_to_execute = [flag2]() {
		flag2->wait();
	};

	//expected behavior once we submit the first job we expect the number of tasks to increment by 1, then 2 ...
	int number_of_tasks = 0;
	int number_of_tasks_in_queue = 0;
	pool->get_number_tasks(number_of_tasks, number_of_tasks_in_queue);
	std::string message = "Number of tasks (pending + execution): (" +std::to_string(number_of_tasks) +" + "+ std::to_string(number_of_tasks_in_queue) + ") (expected 0) \n";
	std::cout << message;
	pool->submit(job1);
	pool->get_number_tasks(number_of_tasks, number_of_tasks_in_queue);
	message = "Number of tasks (pending + execution): (" + std::to_string(number_of_tasks) + " + " + std::to_string(number_of_tasks_in_queue) + ") (expected 1) \n";
	std::cout << message;
	pool->submit(job2);
	pool->get_number_tasks(number_of_tasks, number_of_tasks_in_queue);
	message = "Number of tasks (pending + execution): (" + std::to_string(number_of_tasks) + " + " + std::to_string(number_of_tasks_in_queue) + ") (expected 2) \n";
	std::cout << message;
	pool->submit(job3);
	pool->submit(job4);
	pool->get_number_tasks(number_of_tasks, number_of_tasks_in_queue);
	message = "Number of tasks (pending + execution): (" + std::to_string(number_of_tasks) + " + " + std::to_string(number_of_tasks_in_queue) + ") (expected 4) \n";
	std::cout << message;
	flag1->set();
	pool->get_number_tasks(number_of_tasks, number_of_tasks_in_queue);
	message = "Number of tasks (pending + execution): (" + std::to_string(number_of_tasks) + " + " + std::to_string(number_of_tasks_in_queue) + ") (expected <4)\n ";
	std::cout << message;
	flag2->set();
	pool->get_number_tasks(number_of_tasks, number_of_tasks_in_queue);
	message = "Number of tasks (pending + execution): (" + std::to_string(number_of_tasks) + " + " + std::to_string(number_of_tasks_in_queue) + ") (expected <4) \n";
	std::cout << message;
	std::cout << message;
	return 0;
}

struct PoPable {
	int val;

	PoPable() : val{ 0 } {
	
	};


};

void test_thread_safe_queue() {
	PoPable pop;
	curan::utilities::SafeQueue<PoPable> safe_queue;
	auto put_in = [&safe_queue](PoPable pop) {
		safe_queue.push(pop);
	};
	auto function_to_execute = [&safe_queue]() {
		try {
			PoPable temp;
			std::cout << "starting to wait for popable\n";
			while (!safe_queue.is_invalid()) {
				if(safe_queue.wait_and_pop(temp))
					std::cout << "Received a poopable! yeye = " << temp.val << "\n";
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

class TestingCallingMechanism  : std::enable_shared_from_this<TestingCallingMechanism>{
	TestingCallingMechanism() {
	
	}

public:

	void call_functional(float number) {
		std::printf("hello, my name is machine %f\n",number);
	}

	[[no_discard]] static std::shared_ptr<TestingCallingMechanism> make_shared() {
		return std::shared_ptr<TestingCallingMechanism>(new TestingCallingMechanism());
	}
};


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
		auto lambda = [shared_memory_to_control]() {
			return asio::buffer(shared_memory_to_control->data(), shared_memory_to_control->size());
			
		};
		buff_of_interest = curan::utilities::CaptureBuffer::make_shared(lambda);
		//value to constrol is deleted as well as the local copy of the shared_memory_control, but the shared_pointer is
		//stored into the lamda which is then copied into the capture memory buffer
	}
	
	std::cout << "Second buffer is: " << buff_of_interest << "\n";
}

int main() {
	//initualize the thread pool;
	curan::utilities::initialize_thread_pool(10);

	test_shared_flag();
	test_job_and_thread_pool();
	test_thread_safe_queue();
	test_memory_buffers();
	int a = 0;
	std::cout << ++a << '\n';
	//terminate the thread pool;
	curan::utilities::terminate_thread_pool();
	return 0;
}

