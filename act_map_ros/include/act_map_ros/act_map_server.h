#pragma once

#include <act_map/act_map.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>

#include "act_map_ros/common_ros.h"
#include "act_map_msgs/FilePath.h"
#include "act_map_msgs/Layer.h"

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
    double viz_bview_size_vox_ratio_;
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
    std::vector<double> preset_pos_factor_layer_ranges_;

    // occupancy layer preset
    bool preset_occ_layer_;
    std::string occ_points_fn_;
    std::string aver_view_dirs_fn_;

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

  inline act_map::ActMap<T>& getActMapRef()
  {
    return *act_map_;
  }

  inline const act_map::ActMap<T>& getActMapCRef() const
  {
    return *act_map_;
  }

  void updateVisualization() const;
  void saveMap(const std::string& path) const;
  void loadMap(const std::string& path);
  void publishMap() const;
  void recomputeMap();

private:
  // dispatch to different initializaiton functions
  // NOTE: make sure only one is valid for a type
  template <typename ST = T>
  typename std::enable_if<!act_map::traits::is_vis_vox<ST>::value>::type
  typedInit();

  template <typename ST = T>
  typename std::enable_if<act_map::traits::is_gp_vis_vox<ST>::value>::type
  typedInit();

  template <typename ST = T>
  typename std::enable_if<act_map::traits::is_quadpoly_vis_vox<ST>::value>::type
  typedInit();

  void readServerOptions();
  void readParams();
  void init();
  void setupROS();
  void feature3dCallback(const PCLPointCloud::ConstPtr& pc,
                         const size_t cam_idx);
  void updateKernelLayer();

  void bodyPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose);

  void visualizeOccupancy() const;
  void visualizeKernelBlocks() const;
  void visualizeKerBestViews(const bool viz_all = false) const;
  void visualizeAllUponPC(const ros::Time& time);

  // services
  bool recomputeCallBack(std_srvs::Empty::Request& request,
                         std_srvs::Empty::Response& response);

  bool updateVisualizationCallback(std_srvs::Empty::Request& request,
                                   std_srvs::Empty::Response& response);

  bool clearMapCallback(std_srvs::Empty::Request& request,
                        std_srvs::Empty::Response& response);

  bool allocateKernelMapCallback(std_srvs::Empty::Request& request,
                                 std_srvs::Empty::Response& rsponse);

  bool saveMapCallback(act_map_msgs::FilePath::Request& request,
                       act_map_msgs::FilePath::Response& response);

  bool loadMapCallback(act_map_msgs::FilePath::Request& request,
                       act_map_msgs::FilePath::Response& response);

  bool publishMapCallback(std_srvs::Empty::Request& request,
                          std_srvs::Empty::Response& rsponse);

  void kernelLayerCallback(const act_map_msgs::Layer& layer);
  void occupancyLayerCallback(const act_map_msgs::Layer& layer);

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
  ros::Subscriber body_pose_sub_;
  ros::Subscriber pos_factor_layer_sub_;
  ros::Subscriber occ_layer_sub_;

  // publishers
  ros::Publisher occupied_vox_;
  ros::Publisher occ_view_dir_pub_;
  ros::Publisher kernel_blk_centers_pub_;
  ros::Publisher kvox_bestviews_pub_;
  ros::Publisher pub_markers_;
  ros::Publisher occ_layer_pub_;
  ros::Publisher pos_factor_layer_pub_;
  bool pub_kvox_bestview_ = false;
  bool pub_kernel_blk_centers_ = false;

  // services
  ros::ServiceServer recompute_srv_;
  ros::ServiceServer clear_map_srv_;
  ros::ServiceServer update_vis_srv_;
  ros::ServiceServer allocate_ker_map_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;
  ros::ServiceServer publish_map_srv_;
};
using TraceMapServer = ActMapServer<act_map::QuadTraceVoxel>;
using InfoMapServer = ActMapServer<act_map::QuadInfoVoxel>;

using GPTraceMapServer = ActMapServer<act_map::GPTraceVoxel>;
using GPInfoMapServer = ActMapServer<act_map::GPInfoVoxel>;

using QuadPolyTraceMapServer = ActMapServer<act_map::QuadPolyTraceVoxel>;
using QuadPolyInfoMapServer = ActMapServer<act_map::QuadPolyInfoVoxel>;
}  // namespace act_map_ros
#include "act_map_ros/act_map_server_inl.h"
