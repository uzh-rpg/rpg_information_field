#pragma once

#include <Eigen/Core>

namespace act_map
{
namespace internal
{
struct blockEqual
{
  template <typename T, typename ValueT, int r, int c>
  void operator()(T& a, const Eigen::Matrix<ValueT, r, c>& b)
  {
    a = b;
  }
};

struct blockPlusEqual
{
  template <typename T, typename ValueT, int r, int c>
  void operator()(T& a, const Eigen::Matrix<ValueT, r, c>& b)
  {
    a += b;
  }
};

struct blockMinusEqual
{
  template <typename T, typename ValueT, int r, int c>
  void operator()(T& a, const Eigen::Matrix<ValueT, r, c>& b)
  {
    a -= b;
  }
};
}
}
