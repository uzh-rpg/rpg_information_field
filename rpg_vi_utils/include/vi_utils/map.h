//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <string>
#include <unordered_map>
#include <memory>

#include <Eigen/Core>
#include <glog/logging.h>

namespace vi_utils
{

class Map
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  size_t n_points_ = 0;
  Eigen::Matrix<double, 3, Eigen::Dynamic> points_;
  std::vector<bool> is_fixed_;
  std::vector<int> points_ids_;

  // this is useful if the Map stores a subset of all the points that
  // are available in the world frame. Then obs id may be the index of
  // all the world points.
  std::unordered_map<int, int> obs_id_to_map_points_id_;

  // save and load names
  static const std::string kPts;
  static const std::string kMapping;
  static const std::string kExt;

  void resize(const size_t n_pts, const bool is_fixed=false);
  void setPoints(const Eigen::Matrix<double, 3, Eigen::Dynamic>& points,
                 const bool is_fixed = false);

  bool check() const
  {
    CHECK_EQ(n_points_, static_cast<size_t>(points_.cols()));
    CHECK_EQ(n_points_, static_cast<size_t>(is_fixed_.size()));
    CHECK_EQ(n_points_, static_cast<size_t>(points_ids_.size()));
    CHECK_EQ(n_points_, obs_id_to_map_points_id_.size());
  }

  void load(const std::string& abs_map_file,
            const std::string& abs_id_mapping_file);
  void save(const std::string& abs_map_file,
            const std::string& abs_id_mapping_file) const;
  void save(const std::string& abs_dir) const;
  void load(const std::string& abs_dir);
  void disturbPoints(const double dist_pts);
  void getPointIdsEigen(Eigen::Matrix<int, 1, Eigen::Dynamic>* ids_eigen) const;
private:
  void createPointIds();
  void createTrivialIdMapping();
};
using MapPtr = std::shared_ptr<Map>;
}
namespace vi
{
using Map = vi_utils::Map;
using MapPtr = vi_utils::MapPtr;
}
