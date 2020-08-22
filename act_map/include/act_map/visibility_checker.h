#pragma once

#include <act_map/depth_map.h>
#include <vi_utils/cam_min.h>

namespace act_map
{
// Default values: nothing is done
struct VisibilityCheckerOptions
{
  // min and max distances
  double min_dist = 0;
  double max_dist = std::numeric_limits<double>::infinity();

  bool use_view_filtering = false;
  // minimum cosine value (i.e., max angle) for the angle to provided view
  // direction
  double min_view_angle_cos = -1.1;

  bool use_depth_layer_ = false;
  DepthMapOptions dm_options_;
  std::string depth_layer_proto_fn_ = "";

  bool use_camera_ = false;
  std::string cam_dir_ = "";
};

class VisibilityChecker
{
public:
  VisibilityChecker() = delete;

  VisibilityChecker(const VisibilityCheckerOptions& options,
                    const DepthMapPtr& dm_ptr = nullptr,
                    const vi::PinholeCamPtr cam_ptr = nullptr)
    : options_(options), depth_map_(dm_ptr), cam_(cam_ptr)
  {
  }

  static std::shared_ptr<VisibilityChecker>
  createVisibilityChecker(const VisibilityCheckerOptions& options);

  virtual ~VisibilityChecker();

  // simple wrapper around the depth map
  void getPointsVisibilityAt(const rpg::Position& view_pos,
                             const rpg::PositionVec& points,
                             VisStatusVec* vis_status) const;

  // get visible points based on the position: occlusion, viewpoint, depth range
  void getVisibleIdx(const rpg::Position& view_pos,
                     const rpg::PositionVec& points,
                     const rpg::PositionVec& view_dirs_from_pt,
                     VisIdx* vis_idx) const;

  // get visible points: considering occlusion, viewpoint change AND FoV constraint
  void getVisibleIdx(const rpg::Pose& Twc, const rpg::PositionVec& points,
                     const rpg::PositionVec& view_dirs_from_pt,
                     VisIdx* vis_idx) const;

  inline DepthMapPtr& depthMapPtr()
  {
    return depth_map_;
  }

  inline const DepthMapPtr& depthMapPtr() const
  {
    return depth_map_;
  }

  inline vi::PinholeCamPtr& camPtr()
  {
    return cam_;
  }

  inline const vi::PinholeCamPtr& camPtr() const
  {
    return cam_;
  }

  VisibilityCheckerOptions options_;

private:
  DepthMapPtr depth_map_ = nullptr;
  vi::PinholeCamPtr cam_ = nullptr;
};

using VisibilityCheckerPtr = std::shared_ptr<VisibilityChecker>;

}  // namespace act_map
