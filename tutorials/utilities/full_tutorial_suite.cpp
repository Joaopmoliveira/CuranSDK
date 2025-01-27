#include <iostream>

#include "utils/TheadPool.h"

void thread_pool_tutorial()
{
    using namespace curan::utilities;
    std::atomic<size_t> value = 0;
    Job job{"increment value", [&]()
            { ++value; }};
    std::cout << "the description should be the same: (expected \"increment value\")" << job.description() << std::endl;
    std::cout << "the value should be 0: " << value << std::endl;
    job();
    std::cout << "the value should be 1: " << value << std::endl;
    job();
    std::cout << "the value should be 2: " << value << std::endl;
    job();
    std::cout << "the value should be 3: " << value << std::endl;
    job();
    std::cout << "the value should be 4: " << value << std::endl;
    {
        value = 0;
        auto pool = ThreadPool::create(1, TERMINATE_ALL_PENDING_TASKS);
        for (size_t i = 0; i < 100; ++i)
            pool->submit(job);
    } // ~pool() is called here
    std::cout << "the value should be 99: " << value << std::endl;
    {
        value = 0;
        auto pool = ThreadPool::create(1, RETURN_AS_FAST_AS_POSSIBLE);
        for (size_t i = 0; i < 100; ++i)
            pool->submit("increment value", [&]()
                         {std::this_thread::sleep_for(std::chrono::microseconds(1));++value; });
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    } // ~pool() is called here
    std::cout << "the value should be smaller than 99: " << value << std::endl;
}

#include "utils/SafeQueue.h"

void safe_queue_tutorial()
{
    using namespace curan::utilities;
    SafeQueue<std::string> queue;
    auto pool = ThreadPool::create(1, TERMINATE_ALL_PENDING_TASKS);
    pool->submit("string submission", [&]()
                 {std::this_thread::sleep_for(std::chrono::milliseconds(50)); queue.push("osifhm9xq904rsdfsdcvw4tererge55gdfgx0"); });
    auto value = queue.wait_and_pop(std::chrono::milliseconds(100));
    std::cout << (value ? "the string (expected \"osifhm9xq904rsdfsdcvw4tererge55gdfgx0\")in the queue is:" + *value : "no string was received (unexpected)") << std::endl;
    pool->submit("string submission", [&]()
                 {std::this_thread::sleep_for(std::chrono::milliseconds(500)); queue.push("osifhm9xq904rsdfsdcvw4tererge55gdfgx0"); });
    value = queue.wait_and_pop(std::chrono::milliseconds(100));
    std::cout << (value ? "this case should never happen" + *value : "expected no value due to timing") << std::endl;
    value = queue.try_pop();
    std::cout << (value ? "this case should never happen" + *value : "expected no value due to timing") << std::endl;
    queue.emplace("osifhm9xq904rsdfsdcvw4tererge55gdfgx0");
    value = queue.try_pop();
    std::cout << (value ? "the string (expected \"osifhm9xq904rsdfsdcvw4tererge55gdfgx0\")in the queue is:" + *value : "no string was received (unexpected)") << std::endl;
}

#include "utils/MemoryUtils.h"

auto comparator(const std::string &value_to_control, const std::shared_ptr<curan::utilities::MemoryBuffer> &buffer)
{
    size_t address = 0;
    for (auto begin = buffer->begin(); begin != buffer->end(); ++begin)
    {
        for (size_t j = 0; j < begin->size(); ++j, ++address)
            if (value_to_control.at(address) != *(((const char *)begin->data()) + j))
                return false;
    }
    return true;
};

void memory_utils_tutorial()
{
    using namespace curan::utilities;
    {
        std::string mem = "osifhm9xq904rsdfsdcvw4tererge55gdfgx0";
        // the copy buffer receives an pointer and the size of data to be copied and does so promply
        auto buffer = CopyBuffer::make_shared(mem.data(), mem.size());
        std::cout << "the buffers should be the same: " << comparator(mem, buffer) << std::endl;
        mem[0] = 'a';
        std::cout << "the buffers should not be the same: " << comparator(mem, buffer) << std::endl;
    }

    {
        auto mem = std::make_shared<std::string>("osifhm9xq904rsdfsdcvw4tererge55gdfgx0");
        // the copy buffer receives an pointer and the size of data to be copied and does so promply
        auto buffer = CaptureBuffer::make_shared(mem->data(), mem->size(), mem);
        std::cout << "the buffers should be the same: " << comparator(*mem, buffer) << std::endl;
        mem->data()[0] = 'a';
        std::cout << "the buffers should still be the same: " << comparator(*mem, buffer) << std::endl;
    }
}

#include "utils/DateManipulation.h"

void date_manipulation_tutorial()
{
    using namespace curan::utilities;
    auto date = formated_date<std::chrono::system_clock>(std::chrono::system_clock::time_point(std::chrono::system_clock::duration(0)));
    std::cout << "the computed date with the provided time point is (expected \"1970-01-01 00:00:00\"):" << date << std::endl;
}

#include "utils/FileStructures.h"

template <typename T>
T parse_from_file(std::istream &instream)
{
    T file_encoding{instream};
    return file_encoding;
}

std::stringstream print_to_file(auto type)
{
    std::stringstream mock_file_in_disk;
    mock_file_in_disk << type;
    return mock_file_in_disk;
}

void file_structure_tutorial()
{
    using namespace curan::utilities;
    auto creation_data = formated_date<std::chrono::system_clock>(std::chrono::system_clock::now());

    {
        UltrasoundCalibrationData data{creation_data, Eigen::Matrix<double, 4, 4>::Identity(), 0.0};
        std::cout << "(original  ) ultrasound calibration data: \n"
                  << data << std::endl;
        auto mock_file_in_disk = print_to_file(data);
        auto copydata = parse_from_file<UltrasoundCalibrationData>(mock_file_in_disk);
        std::cout << "(replication) ultrasound calibration data: \n"
                  << copydata << std::endl;
    }

    {
        NeedleCalibrationData data{creation_data, Eigen::Matrix<double, 4, 4>::Identity(), 0.0};
        std::cout << "(original  ) needle calibration data: \n"
                  << data << std::endl;
        auto mock_file_in_disk = print_to_file(data);
        auto copydata = parse_from_file<NeedleCalibrationData>(mock_file_in_disk);
        std::cout << "(replication) needle calibration data: \n"
                  << copydata << std::endl;
    }

    {
        RegistrationData data{creation_data, Eigen::Matrix<double, 4, 4>::Identity(), 0.0, Type::VOLUME};
        std::cout << "(original  ) registration data: \n"
                  << data << std::endl;
        auto mock_file_in_disk = print_to_file(data);
        auto copydata = parse_from_file<RegistrationData>(mock_file_in_disk);
        std::cout << "(replication) registration data: \n"
                  << copydata << std::endl;
    }

    {
        TrajectorySpecificationData data{creation_data, Eigen::Matrix<double, 3, 1>::Ones(), Eigen::Matrix<double, 3, 1>::Ones(), Eigen::Matrix<double, 3, 3>::Identity(), "path_to_moving_image"};
        std::cout << "(original  ) trajectory specification data: \n"
                  << data << std::endl;
        auto mock_file_in_disk = print_to_file(data);
        auto copydata = parse_from_file<TrajectorySpecificationData>(mock_file_in_disk);
        std::cout << "(replication) trajectory specification data: \n"
                  << copydata << std::endl;
    }
}

#include "utils/Logger.h"

void logger_tutorial()
{
    using namespace curan::utilities;
    Logger logger{};
    print<Severity::info>("data to print to word{0}\n", 1);
    print<Severity::debug>("data to print to word{0}\n", 2);
    print<Severity::major_failure>("data to print to word{0}\n", 3);
    print<Severity::minor_failure>("data to print to word{0}\n", 4);
    print<Severity::warning>("data to print to word{0}\n", 5);
    for (int i = 0; i < 7 && logger; ++i)
        logger.processing_function();
}

#include "utils/Reader.h"

void reader_tutorial()
{
    using namespace curan::utilities;
    char matrix_correct[] = R"(11.14285714 , 12.14285714
                               13.14285714 , 14.14285714)";
    std::stringstream datastream;
    datastream << matrix_correct;
    Eigen::MatrixXd matrix = convert_matrix(datastream, ',');
    std::cout << "parsed matrix:\n"
              << matrix << std::endl;
}

#include "utils/StringManipulation.h"

void string_manipulation_tutorial()
{
    using namespace curan::utilities;
    std::cout << "data to output is: " << to_string_with_precision(1324.91234910823409, 1) << std::endl;
    std::cout << "data to output is: " << to_string_with_precision(1324.91234910823409, 2) << std::endl;
    std::cout << "data to output is: " << to_string_with_precision(1324.91234910823409, 3) << std::endl;
    std::cout << "data to output is: " << to_string_with_precision(1324.91234910823409, 4) << std::endl;
    std::cout << "data to output is: " << to_string_with_precision(1324.91234910823409, 5) << std::endl;
    std::cout << "data to output is: " << to_string_with_precision(1324.91234910823409, 6) << std::endl;
}

int main()
{
    try
    {
        thread_pool_tutorial();
        safe_queue_tutorial();
        memory_utils_tutorial();
        date_manipulation_tutorial();
        file_structure_tutorial();
        logger_tutorial();
        reader_tutorial();
        string_manipulation_tutorial();
        return 0;
    }
    catch (...)
    {
        return 1;
    }
}