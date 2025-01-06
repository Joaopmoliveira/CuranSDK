#include "utils/DateManipulation.h"
#include <gtest/gtest.h>

TEST(UnitTestDateFormater, DateFormater){
    const std::string expected{"1970-01-01 00:00:00"}; 
    EXPECT_EQ(curan::utilities::formated_date<std::chrono::system_clock>(std::chrono::system_clock::time_point(std::chrono::system_clock::duration(0))), expected);
    const std::string expected2{"2001-09-09 02:46:40"};
    EXPECT_EQ(curan::utilities::formated_date<std::chrono::system_clock>(std::chrono::system_clock::time_point(std::chrono::system_clock::duration(std::chrono::seconds(1000000000)))), expected2);
}