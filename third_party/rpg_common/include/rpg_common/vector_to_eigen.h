#pragma once

#include <vector>
#include <type_traits>

#include <Eigen/Dense>
#include <glog/logging.h>

namespace rpg_common {

// for a single vector:
//    vectorToEigen({in}, out);
// This function uses row-major: each vector stores a row of the matrix.
template <typename Type, int Rows, int Cols>
void vectorToEigen(const std::vector<std::vector<Type>>& in,
                   Eigen::Matrix<Type, Rows, Cols>* out)
{
  CHECK_NOTNULL(out);
  CHECK(!in.empty());

  CHECK(Rows == Eigen::Dynamic || static_cast<int>(in.size()) == Rows)
      << "Row dimension is not consistent.";
  CHECK(Cols == Eigen::Dynamic || static_cast<int>(in[0].size()) == Cols)
      << "Column dimension is not consistent.";
  for (size_t i = 0; i < in.size() - 1; i++)
  {
    CHECK_EQ(in[i].size(), in[i+1].size());
  }
  out->resize(in.size(), in[0].size());

  for (size_t row_i = 0u; row_i < in.size(); ++row_i)
  {
    for (size_t col_i = 0u; col_i < in[row_i].size(); ++col_i)
    {
      (*out)(static_cast<int>(row_i), static_cast<int>(col_i)) =
          in[row_i][col_i];
    }
  }
}

}

namespace rpg = rpg_common;
