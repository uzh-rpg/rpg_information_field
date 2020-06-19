//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <vi_utils/cam_min.h>

#include <vi_utils/map.h>
#include <vi_utils/states.h>
#include <vi_utils/vi_measurements.h>
#include <rpg_common/aligned.h>

#include "act_map/common.h"

namespace act_map
{
class Simulator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Simulator()
  {
  }

  Simulator(const vi::StatesVec& states_vec,
            const vi::MapPtr& map,
            const vi::PinholeCamVec& cams)
    : initialized_(true), states_(states_vec), map_(map), cams_(cams)
  {
  }

  ~Simulator()
  {
  }

  bool isInitialized() const
  {
    return initialized_;
  }

  void initialize(const vi::StatesVec& states_vec,
                  const vi::MapPtr& map,
                  const vi::PinholeCamVec& cams);

  size_t numOfStates() const
  {
    CHECK(initialized_);
    return states_.size();
  }

  size_t numOfCams() const
  {
    CHECK(initialized_);
    return cams_.size();
  }

  const vi::PinholeCam& getCamConstRef(const size_t cam_id) const
  {
    CHECK_LT(cam_id, cams_.size());
    return *(cams_[cam_id]);
  }

  vi::PinholeCamPtr getCamShared(const size_t cam_id)
  {
    CHECK_LT(cam_id, cams_.size());
    return cams_[cam_id];
  }

  void getStatesAt(const size_t idx, vi::States* states) const
  {
    (*states) = states_[idx];
  }

  void getObservationAt(const size_t states_idx,
                        vi::CamMeasurementsVec* cam_meas_vec,
                        Mat3XdVec* points_vec = nullptr) const;

  friend std::ostream& operator<<(std::ostream& out, const Simulator& rhs);

  void initSequentialSim(const int min_obs = 3,
                         const int max_consec_miss = 5);
  bool step();

  // merge points/ids of all cameras
  void getLastWellObserved(Eigen::Matrix3Xd* pts_w,
                           std::vector<int>* ids,
                           Mat2XdVec* obs = nullptr) const;

  // return the result for individual cameras
  void getLastWellObserved(Mat3XdVec* pts_w,
                           PointIdsVec* ids,
                           Mat2XdVec* obs) const;
  void getLastWellObserved(V3dVecVec* pts_w,
                           PointIdsVec* ids,
                           V2dVecVec* obs) const;

  void getLastObservations(vi::CamMeasurementsVec* last_meas,
                           Mat3XdVec* last_pts_w) const;

  void getLastStates(vi::States* states) const;

  size_t nextStateIdx() const
  {
    return next_state_idx_;
  }
  const vi::CamMeasurementsVec& lastMeas()
  {
    return last_meas_;
  }
  const Mat3XdVec& lastPoints3D()
  {
    return last_pts_w_;
  }
  const std::unordered_map<int, int>& getPtIdToNumObsMap()
  {
    return pt_id_to_num_obs_map_;
  }

private:
  bool initialized_ = false;

  vi::StatesVec states_;
  vi::MapPtr map_;
  vi::PinholeCamVec cams_;

  // for sequential simulation
  vi::CamMeasurementsVec last_meas_;
  Mat3XdVec last_pts_w_;
  vi::States last_states_;

  std::unordered_map<int, int> pt_id_to_num_obs_map_;
  std::unordered_map<int, int> pt_id_to_consec_no_obs_map_;
  size_t next_state_idx_;
  int min_well_obs_;
  int max_consec_miss_;
};
}
