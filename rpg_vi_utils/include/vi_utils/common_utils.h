//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace vi_utils
{
template <typename K, typename V>
bool findInMap(const K key, const std::unordered_map<K, V>& map, V* value)
{
  auto it = map.find(key);
  if (it == map.end())
  {
    return false;
  }
  (*value) = it->second;
  return true;
}

template <typename T>
double averVec(const std::vector<T>& err)
{
  T sum(0.0);
  for (auto e : err)
  {
    sum += e;
  }
  return sum / static_cast<double>(err.size());
}
template <typename _Matrix_Type_>
_Matrix_Type_
pseudoInverse(const _Matrix_Type_& a,
              double epsilon = std::numeric_limits<double>::epsilon())
{
  Eigen::JacobiSVD<_Matrix_Type_> svd(
      a, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = epsilon * std::max(a.cols(), a.rows()) *
                     svd.singularValues().array().abs()(0);
  return svd.matrixV() *
         (svd.singularValues().array().abs() > tolerance)
             .select(svd.singularValues().array().inverse(), 0)
             .matrix()
             .asDiagonal() *
         svd.matrixU().adjoint();
}

// non-template functions
std::string getBaseName(const std::string& filename);

void linspace(const double start,
              const double end,
              const size_t N,
              std::vector<double>* samples);

void linspace(const double start,
              const double end,
              const double step,
              std::vector<double>* samples);
}
