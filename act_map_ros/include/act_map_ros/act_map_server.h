//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <act_map/act_map.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include "act_map_ros/common_ros.h"

namespace act_map_ros
{
template <typename T>
class ActMapServer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ActMapServer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ActMapServer() : ActMapServer<T>(ros::NodeHandle(), ros::NodeHandle("~"))
  {
  }

  // kernel layer update mode
  enum class KernelUpdateMode
  {
    kBatch,
    kIncremental,
    kOnDemand
  };

  struct Options
  {
    size_t n_cams_;

    // visualization
    bool viz_occupancy_;
    bool viz_allocated_kernel_blks_;
    bool viz_active_kernel_blks_;
    bool viz_kvox_best_view_;
    int viz_bview_samples_per_side_;
    bool viz_bview_use_sampling_;
    int viz_bview_incremental_every_n_;
    bool viz_bview_last_updated_;
    double viz_bview_size_;
    bool viz_bview_use_unique_id_;
    double viz_cam_marker_size_;
    bool viz_bview_fixed_color_;
    int viz_bview_color_scale_log_n_;

    // names
    std::string features3d_topic_name_;
    std::string body_pose_topic_name_;
    std::vector<std::string> cam_frame_names_;

    // kernel layer preset
    bool preset_kernel_layer_;
    std::vector<double> preset_ker_layer_ranges_;

    // kernel layer update
    KernelUpdateMode ker_update_mode_;
    // in batch mode, recompute the kernel layer when the occupancy layer has
    // more than <thresh> blocks updated.
    double batch_recompute_occ_thresh_;

    // kernel layer expansion
    std::vector<double> kernel_expand_ranges_;
    bool only_activate_nearby_kernel_blks_;
    double kernel_blks_activate_dist_ = 3.0;
    double kernel_expand_dist_thresh_ = 1.0;

    std::string save_dir_;

    void setKernelUpdateMode(const std::string& kupdate)
    {
      if (kupdate == "batch")
      {
        ker_update_mode_ = KernelUpdateMode::kBatch;
      }
      else if (kupdate == "increment")
      {
        ker_update_mode_ = KernelUpdateMode::kIncremental;
      }
      else if (kupdate == "demand")
      {
        ker_update_mode_ = KernelUpdateMode::kOnDemand;
      }
      else
      {
        LOG(FATAL) << "Unknown update mode.";
      }
    }
  };

private:
  void readServerOptions();
  void readParams();
  void init();
  void setupROS();

  void feature3dCallback(const PCLPointCloud::ConstPtr& pc,
                         const size_t cam_idx);
  void updateKernelLayer();

  void keyCmdCallback(const std_msgs::StringConstPtr& key);
  void bodyPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose);

  void visualizeOccupancy() const;
  void visualizeKernelBlocks() const;
  void visualizeKerBestViews() const;
  void visualizeAll(const ros::Time& time);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // map
  act_map::ActMapOptions map_options_;
  std::unique_ptr<act_map::ActMap<T>> act_map_;

  // server
  Options options_;

  // other variables
  std::vector<std::string> cam_frames_;
  size_t inc_step_cnt_ = 0;
  size_t kernel_expand_cnt_ = 0;

  // subscribers
  std::vector<ros::Subscriber> feature_3d_w_sub_;
  ros::Subscriber key_cmd_sub_;
  ros::Subscriber body_pose_sub_;

  // publishers
  ros::Publisher occupied_vox_;
  ros::Publisher kernel_blk_centers_pub_;
  ros::Publisher kvox_bestviews_pub_;
  ros::Publisher pub_markers_;
  bool pub_kvox_bestview_ = false;
  bool pub_kernel_blk_centers_ = false;
};
using TraceMapServer = ActMapServer<act_map::TraceVoxel>;
using InfoMapServer = ActMapServer<act_map::InfoVoxel>;
}
#include "act_map_ros/act_map_server_inl.h"
