#ifndef CURAN_LOGGER_MINE_HEADER_FILE_
#define CURAN_LOGGER_MINE_HEADER_FILE_

// Type your code here, or load an example.
#include <format>
#include <mutex>
#include <deque>
#include <thread>
#include <condition_variable>
#include <iostream>

#define CURAN_WARNING_LEVEL info

namespace curan {
	namespace utilities {

enum Severity
{
    debug,         // this prints everything
    info,          // this prints relevant events
    warning,       // this prints informations that is relevant when debugging but in day to day life is unecessary
    minor_failure, // these are major errors that in principle should always be logged
    major_failure,  // there are crashing errors
    no_print
};

constexpr size_t max_number_of_strings = 50;

struct Logger;

namespace detail{
    extern Logger *cout;
}
struct Logger
{
    Logger();

    void processing_function();

    ~Logger();

    volatile bool running;
    volatile bool terminated; 
    std::condition_variable cv;
    std::mutex m_mut;
    std::deque<std::string> m_data_queue;

    template <typename... Args>
    void print(std::format_string<Args...> fmt, Args &&...args)
    {
        {
            std::lock_guard<std::mutex> g{m_mut};
            if (m_data_queue.size() < max_number_of_strings){
                auto str = std::format(fmt,std::forward<Args>(args)...);
                m_data_queue.emplace_back(str);
            }else{
                m_data_queue.pop_front();
                auto str = std::format(fmt,std::forward<Args>(args)...);
                m_data_queue.emplace_back(str);
            }
        }
        cv.notify_one();
    }

    operator bool(){
        std::lock_guard<std::mutex> g{m_mut};
        return running;
    }
};

template <Severity level, typename... Args>
void print(std::format_string<Args...> fmt, Args &&...args)
{
    if constexpr (level >= CURAN_WARNING_LEVEL)
    {
        if (detail::cout)
            detail::cout->print(fmt, std::forward<Args>(args)...);
    } // else the text data is discarded
}

}
}

#endif