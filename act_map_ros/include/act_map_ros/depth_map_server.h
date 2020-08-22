#pragma once

#include <act_map/depth_map.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vi_utils/map.h>


namespace act_map_ros
{
class DepthMapServer
{
public:
  DepthMapServer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  DepthMapServer():DepthMapServer(ros::NodeHandle(), ros::NodeHandle("~"))
  {

  }
  struct Options
  {
    std::string depth_layer_proto_fn_;
    std::string feature_points_3d_fn_;
    act_map::DepthMapOptions dm_options_;
    double max_depth_;
  };

  Options options_;

private:
  void readParams();
  void init();
  void setUpROS();

  // visualization
  void resetVizStatus();
  void updateVisualization() const;
  void visualizeSelectedVoxel() const;

  void keyCmdCallback(const std_msgs::StringConstPtr& key);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // map
  act_map::DepthMapPtr dm_ptr_;
  vi::MapPtr points3d_ptr_;

  ros::Subscriber key_cmd_sub_;

  ros::Publisher points3d_pub_;
  ros::Publisher block_centers_pub_;
  ros::Publisher sel_vox_centers_pub_;
  ros::Publisher sel_vox_depths_pub_;
  ros::Publisher markers_pub_;

  // for visualization
  act_map::voxblox::BlockIndexList all_blk_idx_list_;
  size_t viz_vox_idx_;
  size_t viz_blk_idx_;
};

}
