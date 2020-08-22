#include "act_map/visibility_checker.h"

#include <rpg_common/fs.h>

namespace act_map
{
VisibilityChecker::~VisibilityChecker()
{
}

VisibilityCheckerPtr
VisibilityChecker::createVisibilityChecker(const VisibilityCheckerOptions& options)
{
  DepthMapPtr dmap = nullptr;
  if (options.use_depth_layer_)
  {
    CHECK(!options.depth_layer_proto_fn_.empty());
    CHECK(rpg::fs::fileExists(options.depth_layer_proto_fn_));
    dmap.reset(new DepthMap(options.dm_options_));
    dmap->loadDepthLayer(options.depth_layer_proto_fn_);
    VLOG(1) << "Loaded depth map from " << options.depth_layer_proto_fn_;
  }

  vi::PinholeCamPtr cam = nullptr;
  if (options.use_camera_)
  {
    CHECK(!options.cam_dir_.empty());
    CHECK(rpg::fs::pathExists(options.cam_dir_));
    cam = vi::PinholeCam::loadFromDir(options.cam_dir_);
    VLOG(1) << "Loaded camera from " << options.cam_dir_;
    VLOG(1) << *cam;
  }

  return VisibilityCheckerPtr(new VisibilityChecker(options, dmap, cam));
}

void VisibilityChecker::getPointsVisibilityAt(const rpg::Position& view_pos,
                                              const rpg::PositionVec& points,
                                              VisStatusVec* vis_status) const
{
  if (depth_map_)
  {
    depth_map_->queryPointsVisibilityAt(view_pos, points, vis_status);
  }
  else
  {
    vis_status->resize(points.size(), VisStatus::kUnknownDepth);
  }
}

void VisibilityChecker::getVisibleIdx(const rpg::Position& view_pos, const rpg::PositionVec& points,
                                      const rpg::PositionVec& view_dirs_from_pt,
                                      VisIdx* vis_idx) const
{
  CHECK(vis_idx);

  // filter per depth map or nothing
  rpg::PositionVec vec_from_pt_to_view_pos;
  bool filtered_per_depth_map = false;
  VLOG(10) << "> In total: " << points.size() << " points.";
  if (depth_map_)
  {
    const DepthVoxel* vox = depth_map_->getDepthVoxelPtrByCoordinates(view_pos);
    if (vox)
    {
      vox->getVisibleIdxFromPoints(points, vis_idx,
                                   DepthVoxel::getMinMaxD(options_.min_dist, options_.max_dist));
      filtered_per_depth_map = true;
    }
    VLOG(10) << "After DepthMap check: " << vis_idx->size() << " visible points.";
  }
  else
  {
    VLOG(10) << "No depth map, skip filtering by depth.";
  }

  if (!filtered_per_depth_map)
  {
    VLOG(10) << "Use depth range check";
    vis_idx->clear();
    vec_from_pt_to_view_pos.reserve(points.size());
    for (size_t i = 0; i < points.size(); i++)
    {
      vec_from_pt_to_view_pos.emplace_back(view_pos - points[i]);
      const double cur_d = vec_from_pt_to_view_pos.back().norm();
      if (cur_d < options_.max_dist && cur_d > options_.min_dist)
      {
        vis_idx->insert(i);
      }
    }
  }
  VLOG(10) << "After depth range check: " << vis_idx->size() << " visible points.";

  // filter per view directions
  if (!options_.use_view_filtering)
  {
    VLOG(10) << "< Will not use view filtering, return the current result.";
    return;
  }

  if (!view_dirs_from_pt.empty() && !vis_idx->empty())
  {
    CHECK_EQ(view_dirs_from_pt.size(), points.size());
    for (auto it = vis_idx->begin(); it != vis_idx->end();)
    {
      const size_t idx = *it;
      if (std::fabs(view_dirs_from_pt[idx].norm() - 1.0) > 1e-3)
      {
        LOG(WARNING) << "View direction not valid (not norm 1), but view check is enabled.";
        LOG(WARNING) << "Disable this warning by setting use_view_filtering to false"
                        " or provide valid view direction information.";
      }

      Eigen::Vector3d dir_from_pt = view_pos - points[idx];
      dir_from_pt.normalize();

      const double angle_cos = dir_from_pt.dot(view_dirs_from_pt[idx]);
      if (angle_cos > options_.min_view_angle_cos)
      {
        it++;
      }
      else
      {
        it = vis_idx->erase(it);
      }
    }
    VLOG(10) << "< After view check: " << vis_idx->size() << " visible points.";
  }
  else
  {
    VLOG(10) << "< No view directions or no visible points, no checking by view direction.";
  }
}

void VisibilityChecker::getVisibleIdx(const rpg::Pose& Twc, const rpg::PositionVec& points,
                                      const rpg::PositionVec& view_dirs_from_pt,
                                      VisIdx* vis_idx) const
{
  this->getVisibleIdx(Twc.getPosition(), points, view_dirs_from_pt, vis_idx);
  if (vis_idx->empty())
  {
    return;
  }

  if (!cam_)
  {
    LOG_EVERY_N(WARNING, 10) << "No camera found for visibility check.";
    return;
  }

  Eigen::Matrix3Xd pws;
  std::vector<size_t> vis_idx_vec(vis_idx->begin(), vis_idx->end());
  pws.resize(Eigen::NoChange, static_cast<int>(vis_idx_vec.size()));
  for (size_t i = 0; i < vis_idx_vec.size(); i++)
  {
    pws.col(static_cast<int>(i)) = points[vis_idx_vec[i]];
  }
  Eigen::Matrix3Xd pcs = Twc.inverse().transformVectorized(pws);
  Eigen::Matrix2Xd us;
  us.resize(Eigen::NoChange, pws.cols());
  std::vector<bool> is_visible(vis_idx_vec.size());
  cam_->project3dBatch(pcs, &us, &is_visible);

  vis_idx->clear();
  for (size_t i = 0; i < is_visible.size(); i++)
  {
    if (is_visible[i])
    {
      vis_idx->insert(vis_idx_vec[i]);
    }
  }
  VLOG(10) << "After camera projection: " << vis_idx->size() << " visible points.";
}

}  // namespace act_map
