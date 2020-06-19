//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "vi_utils/vi_measurements.h"

#include <algorithm>

namespace vi_utils
{
void CamMeasurements::shuffle()
{
  check();
  std::vector<int> indices(num_keypoints, 0);
  for (int i = 0; i < num_keypoints; i++)
  {
    indices[i] = i;
  }
  std::random_shuffle(indices.begin(), indices.end());

  Eigen::Matrix2Xd new_kpts;
  new_kpts.resize(Eigen::NoChange, num_keypoints);
  std::vector<int32_t> new_global_ids(num_keypoints, 0);
  std::vector<int32_t> new_track_ids(num_keypoints, 0);
  for (int i = 0; i < num_keypoints; i++)
  {
    int new_i = indices[i];
    new_kpts.col(new_i) = keypoints.col(i);
    new_global_ids[new_i] = global_lm_ids[i];
    new_track_ids[new_i] = track_ids[i];
  }
  setMeasurements(new_kpts, new_global_ids, new_track_ids);

  check();
}

bool CamMeasurements::isLandmarkIdVisible(const int32_t lm_id) const
{
  auto it = std::find(global_lm_ids.begin(), global_lm_ids.end(), lm_id);
  return it != global_lm_ids.end();
}

size_t reduceCameraMeasurements(
    const KFCamMeasurementsVec& kf_meas_vec,
    std::set<int32_t>* all_visible_points_ids,
    std::vector<std::vector<int32_t> >* frame_visible_points_ids,
    const int max_fts_per_frame,
    const int min_obs_per_ftr,
    const int cam_id)
{
  CHECK_NOTNULL(all_visible_points_ids);
  CHECK_NOTNULL(frame_visible_points_ids);
  all_visible_points_ids->clear();
  frame_visible_points_ids->clear();

  std::map<int32_t, int> num_obs;

  if (max_fts_per_frame == 0u)
  {
    VLOG(1) << "Will not limiting the number of features!";
  }

  const size_t num_f = kf_meas_vec.size();
  frame_visible_points_ids->resize(num_f);

  size_t total_kps = 0;
  for (size_t i = 0; i < num_f; i++)
  {
    std::vector<int32_t> raw_visible_lm_ids =
        (kf_meas_vec[i])[cam_id].global_lm_ids;
    std::vector<int32_t>& cur_visible_lm_ids = (*frame_visible_points_ids)[i];

    if (max_fts_per_frame == 0 ||
        raw_visible_lm_ids.size() <= max_fts_per_frame)
    {
      cur_visible_lm_ids.insert(cur_visible_lm_ids.begin(),
                                raw_visible_lm_ids.begin(),
                                raw_visible_lm_ids.end());
    }
    else
    {
      // prefer to keep already observed landmarks
      std::remove_if(raw_visible_lm_ids.begin(),
                     raw_visible_lm_ids.end(),
                     [all_visible_points_ids](int32_t id)
                     {
                       return all_visible_points_ids->count(id) == 0;
                     });
      cur_visible_lm_ids.insert(cur_visible_lm_ids.begin(),
                                raw_visible_lm_ids.begin(),
                                raw_visible_lm_ids.begin() + max_fts_per_frame);
    }
    all_visible_points_ids->insert(cur_visible_lm_ids.begin(),
                                   cur_visible_lm_ids.end());
    total_kps += cur_visible_lm_ids.size();
    for (const int32_t id : cur_visible_lm_ids)
    {
      auto iter = num_obs.find(id);
      if (iter != num_obs.end())
      {
        iter->second += 1;
      }
      else
      {
        num_obs[id] = 1;
      }
    }
  }
  VLOG(1) << "There are " << all_visible_points_ids->size() << " points"
          << " and " << total_kps << " observations"
          << " after limiting the features each frame.";

  // check the number of observations
  for (std::set<int32_t>::iterator iter = all_visible_points_ids->begin();
       iter != all_visible_points_ids->end();)
  {
    std::map<int32_t, int>::iterator map_iter = num_obs.find(*iter);
    CHECK(map_iter != num_obs.end());
    if (map_iter->second < min_obs_per_ftr)
    {
      iter = all_visible_points_ids->erase(iter);
    }
    else
    {
      iter++;
    }
  }

  total_kps = 0;
  for (std::vector<int32_t>& f_vis_pts : *frame_visible_points_ids)
  {
    f_vis_pts.erase(std::remove_if(f_vis_pts.begin(),
                                   f_vis_pts.end(),
                                   [&num_obs, &min_obs_per_ftr](int32_t id)
                                   {
                                     std::map<int32_t, int>::iterator map_iter =
                                         num_obs.find(id);
                                     CHECK(map_iter != num_obs.end());
                                     return map_iter->second < min_obs_per_ftr;
                                   }),
                    f_vis_pts.end());
    total_kps += f_vis_pts.size();
  }

  VLOG(1) << "Keeping only well observed features, "
          << "there are " << all_visible_points_ids->size() << " points"
          << " and " << total_kps << " observations"
          << " after limiting the features each frame.";

  return total_kps;
}

void extractGoodLandmarks(const KFMeasVec& data,
                          const int min_obs,
                          const rpg::Matrix3X& all_lms,
                          const std::vector<int>& cam_ids,
                          std::unordered_map<int, int>* global_id_to_good_id,
                          rpg::Matrix3X* good_lms)
{
  CHECK_NOTNULL(global_id_to_good_id);
  CHECK_NOTNULL(good_lms);

  std::vector<int> all_lms_obs(all_lms.cols(), 0);

  // collect the observation statistics
  for (const vi::KFMeasurements& cur_data : data)
  {
    for (const int cam_id : cam_ids)
    {
      const std::vector<int32_t>& vis_ids =
          cur_data.cam_measurements[cam_id].global_lm_ids;
      for (size_t i = 0; i < vis_ids.size(); i++)
      {
        all_lms_obs[vis_ids[i]]++;
      }
    }
  }

  // count and keep the good one
  int good_cnt = std::count_if(all_lms_obs.begin(),
                               all_lms_obs.end(),
                               [&min_obs](int i)
                               {
                                 return i >= min_obs;
                               });
  good_lms->resize(Eigen::NoChange, good_cnt);
  int fill_cnt = 0;
  int nbr_obs = 0;
  for (size_t global_id = 0; global_id < all_lms_obs.size(); global_id++)
  {
    const int num_obs = all_lms_obs[global_id];
    if (num_obs >= min_obs)
    {
      good_lms->block<3, 1>(0, fill_cnt) = all_lms.block<3, 1>(0, global_id);
      global_id_to_good_id->emplace(static_cast<int>(global_id), fill_cnt);
      fill_cnt++;
      nbr_obs += num_obs;
    }
  }
  CHECK_EQ(fill_cnt, good_cnt);
  VLOG(1) << "Extract " << fill_cnt << " points with at least " << min_obs
          << " observations from " << all_lms.cols() << " points, "
          << "and there are " << nbr_obs << " observations in total.";
}
}
