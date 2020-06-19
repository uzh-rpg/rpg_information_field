//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "vi_utils/cam_min.h"

#include <rpg_common/fs.h>
#include <rpg_common/load.h>
#include <rpg_common/save.h>
#include <rpg_common/eigen_type.h>

namespace vi_utils
{
const std::string PinholeCam::kGeo = "/cam_geo";
const std::string PinholeCam::kTbc = "/Tbc";
const std::string PinholeCam::kExt = ".txt";

PinholeCam::PinholeCam(const std::vector<double>& geo_params_vec,
                       const rpg_common::Pose& Tbc)
  : T_b_c_(Tbc)
{
  CHECK_EQ(geo_params_vec.size(), 6);
  fx_ = geo_params_vec[0];
  fy_ = geo_params_vec[1];
  cx_ = geo_params_vec[2];
  cy_ = geo_params_vec[3];
  w_ = geo_params_vec[4];
  h_ = geo_params_vec[5];

  setMargin(0.01);

  updateK();
}

std::ostream& operator<<(std::ostream& os, const PinholeCam& pc)
{
  os << "A minimal pinhole camera model:\n";
  os << "-- size (width height): " << pc.w_ << ", " << pc.h_ << std::endl;
  os << "-- K\n " << pc.K_ << std::endl;
  os << "-- Tbc\n " << pc.T_b_c_ << std::endl;
  return os;
}

void PinholeCam::updateK()
{
  K_.setIdentity();
  K_(0, 0) = fx_;
  K_(0, 2) = cx_;
  K_(1, 1) = fy_;
  K_(1, 2) = cy_;
}

PinholeCamPtr PinholeCam::loadFromFile(const std::string& abs_cam_geo,
                                       const std::string& abs_Tbc)
{
  CHECK(rpg::fs::fileExists(abs_cam_geo));
  CHECK(rpg::fs::fileExists(abs_Tbc));

  rpg::Matrix16 geo_params;
  rpg::load(abs_cam_geo, &geo_params, ' ');
  rpg::Matrix44 T_b_c_mat;
  rpg::load(abs_Tbc, &T_b_c_mat, ' ');

  std::vector<double> geo_params_vec(6);
  for (size_t i = 0; i < 6; i++)
  {
    geo_params_vec[i] = geo_params(0, i);
  }
  rpg::Pose T_b_c(T_b_c_mat);

  return std::make_shared<PinholeCam>(geo_params_vec, T_b_c);
}

PinholeCamPtr PinholeCam::loadFromDir(const std::string& dir)
{
  return loadFromFile(dir + kGeo + kExt, dir + kTbc + kExt);
}

void PinholeCam::saveToFile(const std::string& abs_cam_geo,
                            const std::string& abs_Tbc) const
{
  rpg::Matrix16 geo_params;
  geo_params(0, 0) = fx_;
  geo_params(0, 1) = fy_;
  geo_params(0, 2) = cx_;
  geo_params(0, 3) = cy_;
  geo_params(0, 4) = w_;
  geo_params(0, 5) = h_;
  rpg::save(abs_cam_geo, geo_params);

  rpg::save(abs_Tbc, T_b_c_.getTransformationMatrix());
}

bool PinholeCam::project3d(const Eigen::Vector3d& p_c, Eigen::Vector2d* u) const
{
  Eigen::Vector3d u_homo = K_ * p_c;
  double ux = u_homo(0) / u_homo(2);
  double uy = u_homo(1) / u_homo(2);
  if (u != nullptr)
  {
    (*u)(0) = ux;
    (*u)(1) = uy;
  }
  return isDepthValid(p_c) && isDistanceValid(p_c) &&
         isInsideImage(Eigen::Vector2d(ux, uy));
}

void NCamera::projectBatchWithIds(const StatesVec& states_vec,
                                  const PinholeCamVec& cam_vec,
                                  const Map& map,
                                  KFCamMeasurementsVec* states_meas)
{
  CHECK(!cam_vec.empty());
  CHECK(!states_vec.empty());
  CHECK_NOTNULL(states_meas);

  const size_t n_states = states_vec.size();
  states_meas->resize(n_states);
  const int n_cam = cam_vec.size();
  Eigen::Matrix<int, 1, Eigen::Dynamic> ids;
  map.getPointIdsEigen(&ids);

  for (int states_idx = 0; states_idx < n_states; states_idx++)
  {
    const vi_utils::States& states_i = states_vec[states_idx];
    vi_utils::CamMeasurementsVec& kf_meas = (*states_meas)[states_idx];
    kf_meas.resize(n_cam);
    for (int cam_idx = 0; cam_idx < n_cam; cam_idx++)
    {
      kf_meas[cam_idx].timestamp_ns = states_i.time_ns;
      rpg::Pose T_c_w = (states_i.T_0_cur * cam_vec[cam_idx]->Tbc()).inverse();
      Eigen::Matrix3Xd kf_pcs = T_c_w.transformVectorized(map.points_);
      cam_vec[cam_idx]->project3dBatchWithIds(kf_pcs, ids, &(kf_meas[cam_idx]));
    }
  }
}

int NCamera::numOfCameras(const std::string& abs_dir)
{
  int n = 0;
  while (1)
  {
    const std::string sufn = std::to_string(n);
    if (rpg::fs::fileExists(abs_dir + PinholeCam::kGeo + sufn +
                            PinholeCam::kExt))
    {
      n++;
    }
    else
    {
      break;
    }
  }

  return n;
}

void NCamera::loadCamerasFromDir(const std::string& load_dir,
                                 vi_utils::PinholeCamVec* cams)
{
  CHECK(rpg::fs::pathExists(load_dir));
  CHECK_NOTNULL(cams);

  int n_cam = NCamera::numOfCameras(load_dir);
  for (int cam_id = 0; cam_id < n_cam; cam_id++)
  {
    std::string id_str = std::to_string(cam_id);
    cams->emplace_back(PinholeCam::loadFromFile(
        load_dir + PinholeCam::kGeo + id_str + PinholeCam::kExt,
        load_dir + PinholeCam::kTbc + id_str + PinholeCam::kExt));
  }
}

}  // namespace vi_utils
