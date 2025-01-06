#include <gtest/gtest.h>
#include "utils/StringManipulation.h"

TEST(UnitTestStringManipulation, StringValidation){
  std::string val = curan::utilities::to_string_with_precision(0.142857142857142849212692681248881854116916656494140625, 1);
  EXPECT_EQ(val, "0.1");
  val = curan::utilities::to_string_with_precision(0.142857142857142849212692681248881854116916656494140625, 2);
  EXPECT_EQ(val, "0.14");
  val = curan::utilities::to_string_with_precision(0.142857142857142849212692681248881854116916656494140625, 3);
  EXPECT_EQ(val, "0.143");
  val = curan::utilities::to_string_with_precision(0.142857142857142849212692681248881854116916656494140625, 4);
  EXPECT_EQ(val, "0.1429");
  val = curan::utilities::to_string_with_precision(0.142857142857142849212692681248881854116916656494140625, 5);
  EXPECT_EQ(val, "0.14286");
  val = curan::utilities::to_string_with_precision(11.142857142857142849212692681248881854116916656494140625, 1);
  EXPECT_EQ(val, "11.1");
  val = curan::utilities::to_string_with_precision(11.142857142857142849212692681248881854116916656494140625, 2);
  EXPECT_EQ(val, "11.14");
  val = curan::utilities::to_string_with_precision(11.142857142857142849212692681248881854116916656494140625, 3);
  EXPECT_EQ(val, "11.143");
  val = curan::utilities::to_string_with_precision(11.142857142857142849212692681248881854116916656494140625, 4);
  EXPECT_EQ(val, "11.1429");
  val = curan::utilities::to_string_with_precision(11.142857142857142849212692681248881854116916656494140625, 5);
  EXPECT_EQ(val, "11.14286");
  val = curan::utilities::to_string_with_precision(-0.142857142857142849212692681248881854116916656494140625, 1);
  EXPECT_EQ(val, "-0.1");
  val = curan::utilities::to_string_with_precision(-0.142857142857142849212692681248881854116916656494140625, 2);
  EXPECT_EQ(val, "-0.14");
  val = curan::utilities::to_string_with_precision(-0.142857142857142849212692681248881854116916656494140625, 3);
  EXPECT_EQ(val, "-0.143");
  val = curan::utilities::to_string_with_precision(-0.142857142857142849212692681248881854116916656494140625, 4);
  EXPECT_EQ(val, "-0.1429");
  val = curan::utilities::to_string_with_precision(-0.142857142857142849212692681248881854116916656494140625, 5);
  EXPECT_EQ(val, "-0.14286");
  val = curan::utilities::to_string_with_precision(-11.142857142857142849212692681248881854116916656494140625, 1);
  EXPECT_EQ(val, "-11.1");
  val = curan::utilities::to_string_with_precision(-11.142857142857142849212692681248881854116916656494140625, 2);
  EXPECT_EQ(val, "-11.14");
  val = curan::utilities::to_string_with_precision(-11.142857142857142849212692681248881854116916656494140625, 3);
  EXPECT_EQ(val, "-11.143");
  val = curan::utilities::to_string_with_precision(-11.142857142857142849212692681248881854116916656494140625, 4);
  EXPECT_EQ(val, "-11.1429");
  val = curan::utilities::to_string_with_precision(-11.142857142857142849212692681248881854116916656494140625, 5);
  EXPECT_EQ(val, "-11.14286");
}
