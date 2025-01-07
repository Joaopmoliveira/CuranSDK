#include <gtest/gtest.h>
#include "utils/TheadPool.h"

TEST(UnitTestJobsAndPools, JobsValidation)
{
    size_t value = 0;
    curan::utilities::Job job{"increment value",[&](){++value;}};
    ASSERT_EQ(job.description(), "increment value") << "the description should be the same";
    ASSERT_EQ(value, 0) << "the value should be zero";
    job();
    ASSERT_EQ(value, 1) << "the value should be zero";
    job();
    ASSERT_EQ(value, 2) << "the value should be zero";
    job();
    ASSERT_EQ(value, 3) << "the value should be zero";
    job();
    ASSERT_EQ(value, 4) << "the value should be zero";
    job();
}

TEST(UnitTestJobsAndPools, ThreadPoolValidation)
{
    std::mutex cv_m;
    std::condition_variable_any cv;
    int i = 0;

    size_t value = 0;
    auto pool = curan::utilities::ThreadPool::create(1);
    ASSERT_EQ(value, 0) << "the value should be zero";
    pool->submit("",[&](){
        {
            std::lock_guard<std::mutex> lk(cv_m);
            ++value;
            i = 1;
        }
        cv.notify_all();
    });

    {
        std::unique_lock<std::mutex> lk(cv_m);
        cv.wait(lk, [&]{ return i == 1; });
    }


    ASSERT_EQ(value, 1) << "the value should be one";

    pool->submit("",[&](){
        i = 100;
    });

    pool->submit(curan::utilities::Job{"",[&](){
        {
            std::lock_guard<std::mutex> lk(cv_m);
            ++value;
            i = 2;
        }
        cv.notify_all();
    }});

    {
        std::unique_lock<std::mutex> lk(cv_m);
        cv.wait(lk, [&]{ return i == 2; });
    }

    ASSERT_EQ(value, 2) << "the value should be two";
}