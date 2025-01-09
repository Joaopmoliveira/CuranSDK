#include <gtest/gtest.h>
#include "utils/SafeQueue.h"

TEST(UnitTestSafeQueue, FrontAndTryPopTest) {
  curan::utilities::SafeQueue<double> queue_of_doubles;
  auto val = queue_of_doubles.front();
  ASSERT_EQ(val,std::nullopt) << "the safe container returns empty optional if no values are present";

  queue_of_doubles.push(5.0);
  val = queue_of_doubles.front();
  ASSERT_NE(val,std::nullopt) << "the safe should return a filled optional";
  ASSERT_EQ(*val,5.0) << "the contained object should be equal to 5.0";

  val = queue_of_doubles.try_pop();
  ASSERT_EQ(val,5.0) << "because front was used the contained object should still be five";

  val = queue_of_doubles.front();
  ASSERT_EQ(val,std::nullopt) << "because try pop was used before contained object should be nullopt";

}

TEST(UnitTestSafeQueue, OrderingConstraints) {
  curan::utilities::SafeQueue<double> queue_of_doubles;
  queue_of_doubles.push(1.0);
  queue_of_doubles.push(2.0);
  queue_of_doubles.push(3.0);
  queue_of_doubles.push(4.0);

  auto val = queue_of_doubles.try_pop();
  ASSERT_EQ(val,1.0) << "given the order [1.0 2.0 3.0 4.0] the expected value is 1.0";
  val = queue_of_doubles.try_pop();
  ASSERT_EQ(val,2.0) << "given the order [2.0 3.0 4.0] the expected value is 2.0";
  val = queue_of_doubles.try_pop();
  ASSERT_EQ(val,3.0) << "given the order [3.0 4.0] the expected value is 3.0";
  val = queue_of_doubles.try_pop();
  ASSERT_EQ(val,4.0) << "given the order [4.0] the expected value is 4.0";
  val = queue_of_doubles.try_pop();
  ASSERT_EQ(val,std::nullopt) << "given the order [] the expected value is std::nullopt";
}

TEST(UnitTestSafeQueue, QueueCopyConstraints) {
  curan::utilities::SafeQueue<double> queue_of_doubles;
  queue_of_doubles.push(1.0);
  queue_of_doubles.push(2.0);
  queue_of_doubles.push(3.0);
  queue_of_doubles.push(4.0);

  auto val = queue_of_doubles.request_current_queue_copy();
  std::queue<double> expected_queue{{1.0,2.0,3.0,4.0}};
  ASSERT_EQ(val,expected_queue) << "the internal queues should have the elements in the following order";
}

TEST(UnitTestSafeQueue, Emptyness) {
  curan::utilities::SafeQueue<double> queue_of_doubles;
  queue_of_doubles.push(1.0);
  queue_of_doubles.push(2.0);
  queue_of_doubles.push(3.0);
  queue_of_doubles.push(4.0);

  auto valqueue = queue_of_doubles.request_current_queue_copy();
  std::queue<double> expected_queue{{1.0,2.0,3.0,4.0}};
  ASSERT_EQ(valqueue,expected_queue) << "the internal queues should have the elements in the following order";

  queue_of_doubles.clear();
  auto val = queue_of_doubles.try_pop();
  ASSERT_EQ(val,std::nullopt) << "given the order [] the expected value is std::nullopt";
}

TEST(UnitTestSafeQueue, SizeCheck) {
  curan::utilities::SafeQueue<double> queue_of_doubles;
  for(size_t val = 0; val < 100; ++val)
    queue_of_doubles.push(1.0);
  
  ASSERT_EQ(queue_of_doubles.size(),100) << "there should be five elements inside the queue";
}

TEST(UnitTestSafeQueue, InvalidationCheck) {
  curan::utilities::SafeQueue<double> queue_of_doubles;
  for(size_t val = 0; val < 100; ++val)
    queue_of_doubles.push(1.0);
  ASSERT_EQ(queue_of_doubles.invalid(),false) << "the container should not be invalid";
  ASSERT_EQ(queue_of_doubles.valid(),true) << "the container should be valid";
  queue_of_doubles.invalidate();
  ASSERT_EQ(queue_of_doubles.invalid(),true) << "the container should be invalid";
  ASSERT_EQ(queue_of_doubles.valid(),false) << "the container should not be valid";
  ASSERT_EQ(queue_of_doubles.size(),100) << "the size of queues should be 100";
}