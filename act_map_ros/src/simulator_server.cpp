//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map_ros/simulator_server.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>

#include <rpg_common_ros/params_helper.h>
#include <rpg_common_ros/publish.h>

#include "act_map_ros/conversion_ros.h"

namespace act_map_ros
{
SimulatorServer::SimulatorServer(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  readParams();
  initSimulator();
  std::cout << "Created simulator:\n" << *sim_;
  setupROS();

  ros::Duration(1.5).sleep();
}

void SimulatorServer::readParams()
{
  abs_csv_traj_fn_ =
      rpg_ros::param<std::string>(pnh_, "abs_traj_fn", "traj.csv");
  abs_map_fn_ = rpg_ros::param<std::string>(pnh_, "abs_map_fn", "map.txt");
  abs_cam_dir_ = rpg_ros::param<std::string>(pnh_, "abs_cam_dir", "calib");
  sim_dt_sec_ = rpg_ros::param(pnh_, "sim_dt_sec", 0.02);
  min_obs_ = rpg_ros::param(pnh_, "min_obs", 5);
  max_consec_miss_ = rpg_ros::param(pnh_, "max_consec_miss", 5);
  loop_sim_ = rpg_ros::param(pnh_, "loop_sim", false);

  cam_min_depth_ = rpg_ros::param(pnh_, "cam_min_depth", 0.1);
  cam_max_depth_ = rpg_ros::param(pnh_, "cam_max_depth", 5.0);

  // topic names
  good_pts_topic_name_ =
      rpg_ros::param<std::string>(pnh_, "good_pts_topic_name", "well_obs_pts");
  vis_pts_topic_name_ =
      rpg_ros::param<std::string>(pnh_, "vis_pts_topic_name", "vis_pts");
  img_topic_name_ = rpg_ros::param<std::string>(pnh_, "img_topic_name", "img");
  body_pose_topic_name_ =
      rpg_ros::param<std::string>(pnh_, "body_pose_topic_name", "Twb");
}

void SimulatorServer::initSimulator()
{
  vi::StatesVec states_vec;
  vi::States::load(abs_csv_traj_fn_, &states_vec, ',', true);
  vi::MapPtr map = std::make_shared<vi::Map>();
  map->load(abs_map_fn_, "");
  vi::PinholeCamVec cameras;
  vi::NCamera::loadCamerasFromDir(abs_cam_dir_, &cameras);
  cam_frames_.resize(cameras.size());
  for (size_t i = 0; i < cameras.size(); i++)
  {
    createCamFrameId(i, &cam_frames_[i]);
    cameras[i]->setDepthRange(cam_min_depth_, cam_max_depth_);
  }

  sim_ = std::unique_ptr<act_map::Simulator>(
      new act_map::Simulator(states_vec, map, cameras));

  sim_->initSequentialSim(min_obs_);
}

void SimulatorServer::setupROS()
{
  image_transport::ImageTransport it(pnh_);
  for (size_t cam_idx = 0; cam_idx < sim_->numOfCams(); cam_idx++)
  {
    const std::string suf = std::to_string(cam_idx);
    img_pubs_.emplace_back(it.advertise(img_topic_name_ + suf, 5));
    vis_pts_pubs_.emplace_back(
        pnh_.advertise<PCLPointCloud>(vis_pts_topic_name_ + suf, 10));
    well_obs_pts_pubs_.emplace_back(
        pnh_.advertise<PCLPointCloud>(good_pts_topic_name_ + suf, 10));
  }
  body_pose_pub_ =
      pnh_.advertise<geometry_msgs::PoseStamped>(body_pose_topic_name_, 10);
  key_cmd_sub_ = nh_.subscribe<std_msgs::String>(
      "/sim_cmd", 10, &SimulatorServer::keyCmdCallback, this);
}

void SimulatorServer::keyCmdCallback(const std_msgs::StringConstPtr& key)
{
  const std::string& data = key->data;
  if (data == "c")
  {
    pause_ = false;
    VLOG(1) << "continue simulation!";
  }
  else if (data == "s")
  {
    pause_ = true;
    VLOG(1) << "pause simualtion.";
  }
  VLOG(1) << "unknown key";
}

bool SimulatorServer::stepAndPublish()
{
  if (pause_)
  {
    VLOG(1) << "Simulation paused.";
    return true;
  }

  bool res = sim_->step();
  if (!res)
  {
    if (loop_sim_)
    {
      VLOG(1) << "restart simulation";
      sim_->initSequentialSim(min_obs_);
      return true;
    }
    return false;
  }

  // publish
  vi::CamMeasurementsVec cam_meas;
  act_map::Mat3XdVec vis_pts_w;
  sim_->getLastObservations(&cam_meas, &vis_pts_w);
  act_map::Mat3XdVec well_obs_pts_w;
  act_map::Mat2XdVec well_obs_keypoints;
  sim_->getLastWellObserved(&well_obs_pts_w, nullptr, &well_obs_keypoints);
  vi::States states;
  sim_->getLastStates(&states);
  const rpg::Pose T_w_b = states.T_0_cur;
  ros::Time now = ros::Time().now();

  for (size_t cam_idx = 0; cam_idx < sim_->numOfCams(); cam_idx++)
  {
    const rpg::Pose T_w_c = T_w_b * sim_->getCamConstRef(cam_idx).Tbc();
    rpg_ros::publishTf(T_w_c, now, kWorldFrame, cam_frames_[cam_idx]);
    publishLastImageAt(
        cam_idx, now, cam_meas[cam_idx], well_obs_keypoints[cam_idx]);
    publishPointCloudAt(
        cam_idx, now, vis_pts_w[cam_idx], well_obs_pts_w[cam_idx]);
  }

  geometry_msgs::PoseStamped body_pose_msg;
  body_pose_msg.header.frame_id = kWorldFrame;
  body_pose_msg.header.stamp = now;
  kindrToROSPose(T_w_b, &(body_pose_msg.pose));
  body_pose_pub_.publish(body_pose_msg);

  return true;
}

void SimulatorServer::publishLastImageAt(
    const size_t cam_idx,
    const ros::Time& time,
    const vi::CamMeasurements& meas,
    const Eigen::Matrix2Xd& well_obs_us) const
{
  const vi::PinholeCam& c = sim_->getCamConstRef(cam_idx);
  cv::Mat img;

  img.create(static_cast<int>(c.w()), static_cast<int>(c.h()), CV_8UC3);
  img.setTo(cv::Scalar(120, 120, 120));
  drawKeypoints(meas.keypoints, cv::Scalar(255, 0, 0), 2, &img);
  drawKeypoints(well_obs_us, cv::Scalar(0, 255, 0), 2, &img);

  cv_bridge::CvImage img_msg;
  img_msg.header.stamp = time;
  img_msg.header.frame_id = cam_frames_[cam_idx];
  img_msg.image = img;
  img_msg.encoding = sensor_msgs::image_encodings::BGR8;
  img_pubs_.at(cam_idx).publish(img_msg.toImageMsg());
}

void SimulatorServer::publishPointCloudAt(
    const size_t cam_idx,
    const ros::Time& time,
    const Eigen::Matrix3Xd& vis_pts_w,
    const Eigen::Matrix3Xd& well_obs_pts_w) const
{
  pcl::PointCloud<pcl::PointXYZ> vis_pc_pcl;
  eigen3XdToPCLPointCloud(vis_pts_w, &vis_pc_pcl);
  setPCLPointCloud(&vis_pc_pcl, kWorldFrame, time);
  vis_pts_pubs_.at(cam_idx).publish(vis_pc_pcl);

  pcl::PointCloud<pcl::PointXYZ> well_obs_pc_pcl;
  eigen3XdToPCLPointCloud(well_obs_pts_w, &well_obs_pc_pcl);
  setPCLPointCloud(&well_obs_pc_pcl, kWorldFrame, time);
  well_obs_pts_pubs_.at(cam_idx).publish(well_obs_pc_pcl);
}
void SimulatorServer::sleep()
{
  ros::Duration(sim_dt_sec_).sleep();
}
}
