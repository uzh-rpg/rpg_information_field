//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <act_map/simulator.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>

namespace act_map_ros
{
class SimulatorServer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SimulatorServer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  SimulatorServer() : SimulatorServer(ros::NodeHandle(), ros::NodeHandle("~"))
  {
    pause_ = false;
  }

  bool stepAndPublish();
  void sleep();

  virtual ~SimulatorServer()
  {
  }

private:
  void readParams();
  void initSimulator();
  void setupROS();

  void publishLastImageAt(const size_t cam_idx,
                          const ros::Time& time,
                          const vi::CamMeasurements& meas,
                          const Eigen::Matrix2Xd& well_obs_us) const;
  void publishPointCloudAt(const size_t cam_idx,
                           const ros::Time& time,
                           const Eigen::Matrix3Xd& vis_pts_w,
                           const Eigen::Matrix3Xd& well_obs_pts_w) const;
  void keyCmdCallback(const std_msgs::StringConstPtr& key);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // params
  std::string abs_csv_traj_fn_;
  std::string abs_map_fn_;
  std::string abs_cam_dir_;
  double sim_dt_sec_;
  int min_obs_;
  int max_consec_miss_;
  bool loop_sim_;
  double cam_min_depth_;
  double cam_max_depth_;

  // topic names
  std::string good_pts_topic_name_;
  std::string vis_pts_topic_name_;
  std::string img_topic_name_;
  std::string body_pose_topic_name_;

  // simulator
  std::unique_ptr<act_map::Simulator> sim_;
  std::vector<std::string> cam_frames_;

  // publishers
  std::vector<image_transport::Publisher> img_pubs_;
  std::vector<ros::Publisher> vis_pts_pubs_;
  std::vector<ros::Publisher> well_obs_pts_pubs_;
  ros::Publisher body_pose_pub_;

  // control
  ros::Subscriber key_cmd_sub_;
  bool pause_;
};
}
