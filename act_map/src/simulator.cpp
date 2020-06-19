//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map/simulator.h"

#include "act_map/conversion.h"

namespace act_map
{
void Simulator::initialize(const vi::StatesVec& states_vec,
                           const vi::MapPtr& map,
                           const vi::PinholeCamVec& cams)
{
  states_ = states_vec;
  map_ = map;
  cams_ = cams;

  initialized_ = true;
}

std::ostream& operator<<(std::ostream& out, const Simulator& rhs)
{
  out << "Simulator:\n";
  out << "- States from " << rhs.states_[0].time_ns << " ns to "
      << rhs.states_[rhs.states_.size() - 1].time_ns << " ns.\n"
      << "- Map has " << rhs.map_->n_points_ << " points.\n"
      << "- With " << rhs.cams_.size() << " cameras.\n";
  return out;
}

void Simulator::getObservationAt(const size_t states_idx,
                                 vi::CamMeasurementsVec* cam_meas_vec,
                                 Mat3XdVec* pts_vec) const
{
  CHECK_LT(states_idx, states_.size());
  vi::KFCamMeasurementsVec kf_cam_meas;
  vi::NCamera::projectBatchWithIds(
      { states_[states_idx] }, cams_, *map_, &kf_cam_meas);
  (*cam_meas_vec) = kf_cam_meas[0];
  if (pts_vec)
  {
    pts_vec->resize(cam_meas_vec->size());
    for (size_t cam_idx = 0; cam_idx < cam_meas_vec->size(); cam_idx++)
    {
      const vi::CamMeasurements& meas_i = (*cam_meas_vec)[cam_idx];
      meas_i.check();
      Eigen::Matrix3Xd& pts_i = (*pts_vec)[cam_idx];
      pts_i.resize(Eigen::NoChange, meas_i.keypoints.cols());
      for (size_t obs_idx = 0; obs_idx < meas_i.global_lm_ids.size(); obs_idx++)
      {
        pts_i.col(static_cast<int>(obs_idx)) =
            map_->points_.col(meas_i.global_lm_ids[obs_idx]);
      }
    }
  }
}

void Simulator::initSequentialSim(const int min_obs, const int max_consec_miss)
{
  last_meas_.clear();
  last_pts_w_.clear();
  last_states_.time_ns = -1;
  pt_id_to_num_obs_map_.clear();
  pt_id_to_consec_no_obs_map_.clear();
  next_state_idx_ = 0;
  min_well_obs_ = min_obs;
  max_consec_miss_ = max_consec_miss;
}

bool Simulator::step()
{
  if (next_state_idx_ >= states_.size())
  {
    return false;
  }

  last_states_ = states_[next_state_idx_];
  this->getObservationAt(next_state_idx_, &last_meas_, &last_pts_w_);
  next_state_idx_ += 1;
  for (size_t cam_id = 0; cam_id < last_meas_.size(); cam_id++)
  {
    const vi::CamMeasurements& meas_i = last_meas_[cam_id];
    for (size_t obs_id = 0; obs_id < meas_i.global_lm_ids.size(); obs_id++)
    {
      const int32_t lm_id = meas_i.global_lm_ids[obs_id];
      auto it = pt_id_to_num_obs_map_.find(lm_id);
      if (it == pt_id_to_num_obs_map_.end())
      {
        pt_id_to_num_obs_map_.insert({ lm_id, 1 });
      }
      else
      {
        pt_id_to_num_obs_map_[lm_id]++;
      }
    }
  }

  for (int lm_id = 0; lm_id < map_->points_.cols(); lm_id++)
  {
    bool visible = false;
    for (size_t cam_idx = 0; cam_idx < last_meas_.size(); cam_idx++)
    {
      visible = last_meas_[cam_idx].isLandmarkIdVisible(lm_id) || visible;
    }

    if (!visible)
    {
      auto it = pt_id_to_consec_no_obs_map_.find(lm_id);
      if (it == pt_id_to_consec_no_obs_map_.end())
      {
        pt_id_to_consec_no_obs_map_.insert({ lm_id, 1 });
      }
      else
      {
        pt_id_to_consec_no_obs_map_[lm_id]++;
      }
    }
    else
    {
      pt_id_to_consec_no_obs_map_[lm_id] = 0;
    }
  }

  for (auto v : pt_id_to_consec_no_obs_map_)
  {
    if (v.second >= max_consec_miss_)
    {
      pt_id_to_num_obs_map_.erase(v.first);
    }
  }

  return true;
}

void Simulator::getLastWellObserved(Eigen::Matrix3Xd* pts_w,
                                    std::vector<int>* ids,
                                    Mat2XdVec* obs) const
{
  CHECK_NOTNULL(pts_w);
  CHECK_NOTNULL(ids);

  int well_obs_cnt = 0;
  ids->clear();

  std::vector<int> num_good_obs(numOfCams(), 0);
  if (obs)
  {
    obs->resize(numOfCams());
  }
  for (auto v : pt_id_to_num_obs_map_)
  {
    if (v.second >= min_well_obs_)
    {
      bool visible = false;

      for (size_t cam_idx = 0; cam_idx < last_meas_.size(); cam_idx++)
      {
        if (last_meas_[cam_idx].isLandmarkIdVisible(v.first))
        {
          visible = true;
          num_good_obs[cam_idx]++;
        }
      }

      if (visible)
      {
        well_obs_cnt++;
        ids->push_back(v.first);
      }
    }
  }

  pts_w->resize(Eigen::NoChange, well_obs_cnt);

  for (int i = 0; i < well_obs_cnt; i++)
  {
    pts_w->col(i) = map_->points_.col((*ids)[static_cast<size_t>(i)]);
  }

  if (obs)
  {
    for (size_t cam_idx = 0; cam_idx < last_meas_.size(); cam_idx++)
    {
      (*obs)[cam_idx].resize(Eigen::NoChange, num_good_obs[cam_idx]);
      int good_obs_cnt = 0;
      const vi::CamMeasurements& meas_i = last_meas_[cam_idx];
      for (size_t obs_idx = 0; obs_idx < meas_i.global_lm_ids.size(); obs_idx++)
      {
        int32_t lm_id = meas_i.global_lm_ids[obs_idx];
        auto it = std::find(ids->begin(), ids->end(), lm_id);
        if (it != ids->end())
        {
          (*obs)[cam_idx].col(good_obs_cnt) =
              meas_i.keypoints.col(static_cast<int>(obs_idx));
          good_obs_cnt++;
        }
      }
      CHECK_EQ(good_obs_cnt, num_good_obs[cam_idx]);
    }
  }
}

void Simulator::getLastWellObserved(Mat3XdVec* pts_w,
                                    PointIdsVec* ids,
                                    Mat2XdVec* obs) const
{
  V3dVecVec* pts_w_vec_ptr = nullptr;
  V3dVecVec pts_w_vec;
  if (pts_w)
  {
    pts_w_vec_ptr = &pts_w_vec;
    pts_w->resize(numOfCams());
  }

  V2dVecVec* obs_vec_ptr = nullptr;
  V2dVecVec obs_vec;
  if (obs)
  {
    obs_vec_ptr = &obs_vec;
    obs->resize(numOfCams());
  }

  getLastWellObserved(pts_w_vec_ptr, ids, obs_vec_ptr);

  if (pts_w)
  {
    for (size_t i = 0; i < numOfCams(); i++)
    {
      VecKVecToEigenKX(pts_w_vec[i], &((*pts_w)[i]));
    }
  }
  if (obs)
  {
    for (size_t i = 0; i < numOfCams(); i++)
    {
      VecKVecToEigenKX(obs_vec[i], &((*obs)[i]));
    }
  }
}

void Simulator::getLastWellObserved(V3dVecVec* pts_w,
                                    PointIdsVec* ids,
                                    V2dVecVec* obs) const
{
  if (pts_w)
  {
    pts_w->resize(numOfCams());
  }
  if (ids)
  {
    ids->resize(numOfCams());
  }
  if (obs)
  {
    obs->resize(numOfCams());
  }

  for (auto v : pt_id_to_num_obs_map_)
  {
    if (v.second < min_well_obs_)
    {
      continue;
    }
    for (size_t cam_idx = 0; cam_idx < last_meas_.size(); cam_idx++)
    {
      const vi::CamMeasurements& meas_i = last_meas_[cam_idx];
      const std::vector<int32_t>& global_ids = meas_i.global_lm_ids;
      auto it = std::find(global_ids.begin(), global_ids.end(), v.first);
      if (it != global_ids.end())
      {
        if (pts_w)
        {
          (*pts_w)[cam_idx].emplace_back(map_->points_.col(v.first));
        }
        if (ids)
        {
          (*ids)[cam_idx].emplace_back(v.first);
        }
        if (obs)
        {
          (*obs)[cam_idx].emplace_back(
              meas_i.keypoints.col(it - global_ids.begin()));
        }
      }
    }  // each cam
  }
}

void Simulator::getLastObservations(vi::CamMeasurementsVec* last_meas,
                                    Mat3XdVec* last_pts_w) const
{
  if (last_meas)
  {
    (*last_meas) = last_meas_;
  }
  if (last_pts_w)
  {
    (*last_pts_w) = last_pts_w_;
  }
}

void Simulator::getLastStates(vi::States* states) const
{
  (*states) = last_states_;
}
}
