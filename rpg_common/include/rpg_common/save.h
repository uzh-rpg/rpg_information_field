//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <fstream>

#include <Eigen/Dense>
#include <glog/logging.h>

namespace rpg_common {

// Usage:
// rpg::save(filename, mat_to_save, rpg::EigenCSVFmt);
// rpg::save(filename, mat_to_save, rpg::EigenSpaceSeparatedFmt);
const static Eigen::IOFormat EigenSpaceSeparatedFmt(
    Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n");

const static Eigen::IOFormat EigenCSVFmt(
    Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");

template <typename Type, int Rows, int Cols>
void save(const std::string file, const Eigen::Matrix<Type, Rows, Cols>& mat,
          const Eigen::IOFormat& io_format=EigenSpaceSeparatedFmt)
{
  std::ofstream out(file);
  CHECK(out.is_open());
  out << mat.format(io_format);
}

}
namespace rpg = rpg_common;
