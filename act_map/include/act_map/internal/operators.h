//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
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
