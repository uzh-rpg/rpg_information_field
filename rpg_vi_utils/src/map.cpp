//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "vi_utils/map.h"

#include <rpg_common/fs.h>
#include <rpg_common/load.h>
#include <rpg_common/save.h>

namespace vi_utils
{
const std::string Map::kPts = "/map_points";
const std::string Map::kMapping = "/map_id_mapping";
const std::string Map::kExt = ".txt";

void Map::createPointIds()
{
  points_ids_.resize(n_points_, -1);
  for (int i = 0; i < n_points_; i++)
  {
    points_ids_[i] = i;
  }
  createTrivialIdMapping();
}

void Map::createTrivialIdMapping()
{
  obs_id_to_map_points_id_.clear();
  for (int idx = 0; idx < n_points_; idx++)
  {
    obs_id_to_map_points_id_.emplace(idx, idx);
  }
}

void Map::resize(const size_t n_pts, const bool is_fixed)
{
  n_points_ = n_pts;
  points_.resize(Eigen::NoChange, n_points_);
  is_fixed_.resize(n_points_, is_fixed);
  createPointIds();
}

void Map::setPoints(const Eigen::Matrix<double, 3, Eigen::Dynamic>& points,
                    const bool is_fixed)
{
  n_points_ = points.cols();
  points_ = points;
  is_fixed_.resize(n_points_, is_fixed);
  createPointIds();
}

void Map::load(const std::string& abs_map_file,
               const std::string& abs_id_mapping_file)
{
  CHECK(rpg::fs::fileExists(abs_map_file));
  Eigen::MatrixXd points;
  rpg::load(abs_map_file, &points);
  CHECK_EQ(points.cols(), 3);
  setPoints(points.transpose());

  if (!abs_id_mapping_file.empty())
  {
    obs_id_to_map_points_id_.clear();
    CHECK(rpg::fs::fileExists(abs_id_mapping_file));
    Eigen::MatrixXd ids_mapping;
    rpg::load(abs_id_mapping_file, &ids_mapping);
    CHECK_EQ(ids_mapping.cols(), 2);
    CHECK_EQ(ids_mapping.rows(), n_points_);
    for (int idx = 0; idx < n_points_; idx++)
    {
      obs_id_to_map_points_id_.emplace(
          static_cast<int>(ids_mapping(idx, 0)),
          static_cast<int>(ids_mapping(idx, 1)));
    }
  }
  else
  {
    VLOG(1) << "No id mapping file found. Using identity mapping.";
  }
  check();
}

void Map::save(const std::string& abs_map_file,
               const std::string& abs_id_mapping_file) const
{
  check();
  std::string path, fn;
  rpg::fs::splitPathAndFilename(abs_map_file, &path, &fn);
  CHECK(rpg::fs::pathExists(path));
  Eigen::MatrixXd t_points = points_.transpose();
  rpg::save(abs_map_file, t_points);

  rpg::fs::splitPathAndFilename(abs_id_mapping_file, &path, &fn);
  CHECK(rpg::fs::pathExists(path));
  Eigen::MatrixXd id_mapping_mat;
  id_mapping_mat.resize(n_points_, 2);
  int cnt = 0;
  for (const std::pair<int, int> &p : obs_id_to_map_points_id_)
  {
    id_mapping_mat(cnt, 0) = p.first;
    id_mapping_mat(cnt, 1) = p.second;
    cnt ++;
  }
  rpg::save(abs_id_mapping_file, id_mapping_mat);
}

void Map::save(const std::string& abs_dir) const
{
  save(abs_dir + kPts + kExt, abs_dir + kMapping + kExt);
}

void Map::load(const std::string& abs_dir)
{
  load(abs_dir + kPts + kExt, abs_dir + kMapping + kExt);
}

void Map::disturbPoints(const double dist_pts)
{
  for (int i = 0; i < is_fixed_.size(); i++)
  {
    if (!is_fixed_[i])
    {
      points_.col(i) = points_.col(i) + dist_pts * Eigen::Vector3d::Random();
    }
  }
}

void Map::getPointIdsEigen(
    Eigen::Matrix<int, 1, Eigen::Dynamic>* ids_eigen) const
{
  CHECK_NOTNULL(ids_eigen);
  rpg::vectorToEigen({ points_ids_ }, ids_eigen);
}
}
