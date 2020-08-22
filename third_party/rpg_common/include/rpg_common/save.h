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
          const Eigen::IOFormat& io_format=EigenSpaceSeparatedFmt,
          const std::string& header_line_without_comment_marker=std::string())
{
  std::ofstream out(file);
  CHECK(out.is_open());
  if (!header_line_without_comment_marker.empty())
  {
    out << "# " <<  header_line_without_comment_marker << "\n";
  }
  out << mat.format(io_format);
}

}
namespace rpg = rpg_common;
