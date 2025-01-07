#include "utils/Reader.h"
#include <gtest/gtest.h>

TEST(UnitTestReader, CheckSpacelimitedMatrix) {

  auto to_stringstream =  [](const char *char_array) {
    std::stringstream ss;
    ss << char_array;
    return ss;
  };

  char matrix_correct[] = R"(12.14285714 , 12.14285714
12.14285714 , 12.14285714)";

  char matrix_incorrect_1[] = R"(13.14285714 , 12.14285714
12.14285714 , 12.14285714)";

  char matrix_incorrect_2[] = R"(12.14285714 , 13.14285714
12.14285714 , 12.14285714)";

  char matrix_incorrect_3[] = R"(12.14285714  12.14285714
13.14285714  12.14285714)";

  char matrix_incorrect_4[] = R"(12.14285714 , 12.14285714
12.14285714 , 13.14285714)";

  char matrix_incorrect_5[] = R"(12.14285714 , 12.14285714 , 12.14285714  
13.14285714)";

  char matrix_incorrect_6[] = R"(12.14285714 
 12.14285714 , 12.14285714 , 13.14285714)";

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> expected_matrix =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(2, 2);
  expected_matrix << 12.142857142, 12.142857142, 12.142857142, 12.142857142;
  auto stream = to_stringstream(matrix_correct);
  auto matrix = curan::utilities::convert_matrix(stream, ',');
  ASSERT_EQ(expected_matrix.isApprox(matrix, 0.00001), true)
      << "#matrix_correct# The matrices should be equal (" << expected_matrix << ")\n(" << matrix << ")";

  stream = to_stringstream(matrix_incorrect_1);
  matrix = curan::utilities::convert_matrix(stream, ',');
  ASSERT_NE(expected_matrix.isApprox(matrix, 0.00001), true)
      << "#matrix_incorrect_1# The matrices should be different(" << expected_matrix << ")\n(" << matrix << ")";

  stream = to_stringstream(matrix_incorrect_2);
  matrix = curan::utilities::convert_matrix(stream, ',');
  ASSERT_NE(expected_matrix.isApprox(matrix, 0.00001), true)
      << "#matrix_incorrect_2# The matrices should be different(" << expected_matrix << ")\n(" << matrix << ")";

  stream = to_stringstream(matrix_incorrect_3);
  matrix = curan::utilities::convert_matrix(stream, ',');
  ASSERT_NE(expected_matrix.isApprox(matrix, 0.00001), true)
      << "#matrix_incorrect_3# The matrices should be different(" << expected_matrix << ")\n(" << matrix << ")";

  stream = to_stringstream(matrix_incorrect_4);
  matrix = curan::utilities::convert_matrix(stream, ',');
  ASSERT_NE(expected_matrix.isApprox(matrix, 0.00001), true)
      << "#matrix_incorrect_4# The matrices should be different(" << expected_matrix << ")\n(" << matrix << ")";

  stream = to_stringstream(matrix_incorrect_5);
  EXPECT_ANY_THROW(curan::utilities::convert_matrix(stream, ','))
      << "there are three cols on the first row and one col in the second row";

  stream = to_stringstream(matrix_incorrect_6);
  EXPECT_ANY_THROW(curan::utilities::convert_matrix(stream, ','))
      << "there is one cols on the first row and three cols in the second row";
}