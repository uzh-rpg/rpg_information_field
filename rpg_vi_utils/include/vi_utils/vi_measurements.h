//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include "vi_utils/vi_basic_types.h"

#include <set>

#include <glog/logging.h>

namespace vi_utils
{
enum class MeasurementType
{
  kImage,
  kAccGyr,
  kNone
};

struct CamMeasurements
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CamMeasurements(const int n) { resize(n); }

  CamMeasurements() : CamMeasurements(0) {}

  CamMeasurements(const int64_t timestamp,
                  const Eigen::Matrix2Xd& kpts,
                  const std::vector<int32_t>& global_landmark_ids,
                  const std::vector<int32_t>& local_track_ids)
    : timestamp_ns(timestamp)
  {
    setMeasurements(kpts, global_landmark_ids, local_track_ids);
  }

  void setMeasurements(const Eigen::Matrix2Xd& kpts,
                       const std::vector<int32_t>& global_landmark_ids,
                       const std::vector<int32_t>& local_track_ids)
  {
    keypoints = kpts;
    num_keypoints = keypoints.cols();
    CHECK_EQ(num_keypoints, static_cast<int>(global_landmark_ids.size()));
    CHECK_EQ(num_keypoints, static_cast<int>(local_track_ids.size()));

    global_lm_ids.resize(num_keypoints);
    track_ids.resize(num_keypoints);
    for (int i = 0; i < num_keypoints; i++)
    {
      global_lm_ids[i] = global_landmark_ids[i];
      track_ids[i] = local_track_ids[i];
    }
  }

  void resize(const int n)
  {
    num_keypoints = n;
    keypoints.resize(Eigen::NoChange, n);
    global_lm_ids.resize(n);
    track_ids.resize(n);
  }

  void check() const
  {
    CHECK_EQ(keypoints.cols(), num_keypoints);
    CHECK_EQ(static_cast<int>(global_lm_ids.size()), num_keypoints);
    CHECK_EQ(static_cast<int>(track_ids.size()), num_keypoints);
  }

  // shuffle the order of the measurments, to avoid possible side effect of
  // sequentially processing the camera measurements
  void shuffle();

  bool isLandmarkIdVisible(const int32_t lm_id) const;

  int64_t timestamp_ns;
  int cam_id = 0;

  int32_t num_keypoints;
  Eigen::Matrix2Xd keypoints;
  // useful for localization
  std::vector<int32_t> global_lm_ids;
  // useful for frame-to-frame tracking (e.g., KLT)
  std::vector<int32_t> track_ids;
};
using CamMeasurementsVec = std::vector<CamMeasurements>;
using KFCamMeasurementsVec = std::vector<CamMeasurementsVec>;

struct KFMeasurements
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int64_t time_ns;
  vi_utils::ImuStamps imu_stamps_ns;
  vi_utils::CamMeasurementsVec cam_measurements;
  vi_utils::ImuAccGyrContainer imu_measurements;
};
using KFMeasVec = std::vector<KFMeasurements>;


// functions to deal with measurements
size_t reduceCameraMeasurements(
    const KFCamMeasurementsVec& kf_meas_vec,
    std::set<int32_t>* all_visible_points_ids,
    std::vector<std::vector<int32_t>>* frame_visible_points_ids,
    const int max_fts_per_frame,
    const int min_obs_per_ftr,
    const int cam_id);

// extract landmarks that are observed at least a few times
void extractGoodLandmarks(
    const KFMeasVec& data,
    const int min_obs,
    const rpg::Matrix3X& all_lms,
    const std::vector<int>& cam_ids,
    std::unordered_map<int, int>* global_id_to_good_id,
    rpg::Matrix3X* good_lms);

}
namespace vi
{
using CamMeasurements = vi_utils::CamMeasurements;
using CamMeasurementsVec = vi_utils::CamMeasurementsVec;
using KFCamMeasurementsVec = vi_utils::KFCamMeasurementsVec;
using KFMeasurements = vi_utils::KFMeasurements;
using KFMeasVec = vi_utils::KFMeasVec;
}
