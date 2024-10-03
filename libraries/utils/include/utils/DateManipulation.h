#ifndef CURAN_DATE_MANIPULATION_HEADER_FILE_
#define CURAN_DATE_MANIPULATION_HEADER_FILE_

#include <iostream>
#include <iomanip>
#include <chrono>
#include <time.h>
#include <ctime>

namespace curan {
namespace utilities {

inline std::string get_formated_date(){
    auto now = std::chrono::system_clock::now();
    std::time_t in_time_t = std::chrono::system_clock::to_time_t(now);
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