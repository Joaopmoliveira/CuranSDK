#ifndef CURAN_DATE_MANIPULATION_HEADER_FILE_
#define CURAN_DATE_MANIPULATION_HEADER_FILE_

#include <iostream>
#include <iomanip>
#include <chrono>
#include <time.h>
#include <ctime>

namespace curan {
namespace utilities {

template<typename clocktype>
inline std::string formated_date(typename clocktype::time_point point){
    static_assert(std::chrono::is_clock_v<clocktype>,"type must be a clock");
    std::time_t in_time_t = clocktype::to_time_t(point);
    std::tm local_tm;
    localtime_s(&local_tm, &in_time_t); 
    char buffer[100]; 
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %X", &local_tm);
    std::string formatted_time(buffer);
    return formatted_time;
}

}
}

#endif