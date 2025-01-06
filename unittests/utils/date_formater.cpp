#include "utils/DateManipulation.h"
#include <gtest/gtest.h>

TEST(UnitTestDateFormater, DateFormater){
    auto current_date = curan::utilities::formated_date<std::chrono::system_clock>(std::chrono::system_clock::now());

}