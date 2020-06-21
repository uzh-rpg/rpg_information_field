#pragma once

#include <fstream>

#include <Eigen/Dense>
#include <glog/logging.h>

#include "rpg_common/vector_to_eigen.h"

namespace rpg_common {

// Does the inverse of:
// Eigen::Matrix x;
// std::ofstream out("filename");
// out << x;
// I.e. loads a space-separated file into a matrix.
template <typename Type, int Rows, int Cols>
void load(const std::string& file, Eigen::Matrix<Type, Rows, Cols>* result,
          const char delim=' ')
{
  CHECK_NOTNULL(result);
  std::ifstream in(file);
  CHECK(in.is_open());

  std::vector<std::vector<Type>> coeffs;
  while (!in.eof())
  {
    std::string line;
    getline(in, line);
    if (line.size() == 0 || line.at(0) == '#')
    {
      continue;
    }

    coeffs.emplace_back();
    std::istringstream iss(line);
    std::string value;
    while (std::getline(iss, value, delim))
    {
      coeffs.back().emplace_back(static_cast<Type>(std::stod(value)));
    }
    std::getline(iss, value, '\n');
    coeffs.back().emplace_back(static_cast<Type>(std::stod(value)));

    if (iss.fail())
    {
      coeffs.back().pop_back();
    }

    if (coeffs.back().empty())
    {
      coeffs.pop_back();
    }
  }

  vectorToEigen(coeffs, result);
}
}  // namespace rpg_common

namespace rpg = rpg_common;
