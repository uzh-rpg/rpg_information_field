#pragma once

#include <voxblox_ros/esdf_server.h>
#include <act_map_ros/act_map_server.h>
#include <act_map/information_potential.h>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include "act_map_exp/PlanConfig.h"

namespace act_map_exp
{
using Point = pcl::PointXYZRGB;
using PointCloud = pcl::PointCloud<Point>;

template <typename T>
class PlannerBase
{
public:
  const static std::string kWorldFrame;
  const static std::string kSaveTwbNm;
  const static std::string kSaveTwcNm;
  const static std::string kSaveTwcUENm;

  PlannerBase() = delete;
  PlannerBase(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  inline virtual void plan()
  {
    LOG(WARNING) << "Base planner plan() function called.";
  }

  inline virtual void visualize()
  {
    LOG(WARNING) << "Base planner visualize() function called.";
  }

  inline virtual void saveResults()
  {
    LOG(WARNING) << "Base planner saveResults() function called.";
  }

  inline virtual void resetPlannerState()
  {
    LOG(WARNING) << "Base planner resetPlannerState() function called";
  }

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  inline virtual bool setPlannerStateCallback(PlanConfig::Request& req,
                                              PlanConfig::Response& res)
  {
    LOG(WARNING) << "Base planner setPlanner() function called";
    return false;
  }

  bool planVisSaveCallback(std_srvs::Empty::Request& req,
                           std_srvs::Empty::Response& res);

  bool resetPlannerCallback(std_srvs::Empty::Request& req,
                            std_srvs::Empty::Response& res);

  bool publishVisualizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);

  // common services
  ros::ServiceServer set_planner_state_srv_;
  ros::ServiceServer plan_vis_save_srv_;
  ros::ServiceServer reset_planner_srv_;
  ros::ServiceServer pub_vis_srv_;

  // basic trajectory and markers
  ros::Publisher traj_orient_pub_;
  ros::Publisher traj_pos_pub_;
  ros::Publisher general_marker_pub_;

  // information potential
  act_map::InfoPotentialPtr<T> info_pot_ptr_;

  //
  std::shared_ptr<voxblox::EsdfServer> esdf_server_;
  std::shared_ptr<act_map_ros::ActMapServer<T>> act_map_server_;

  // services for convenience
  ros::ServiceServer update_vis_srv_;
  ros::ServiceServer clear_map_srv;

  void createInfoPotential();

  bool updateAllVisualizationCallback(std_srvs::Empty::Request& req,
                                      std_srvs::Empty::Response& res);
  bool clearAllMapCallback(std_srvs::Empty::Request& req,
                           std_srvs::Empty::Response& res);

  // utilities
  bool hasValidESDFMap() const;
  bool hasValidInfoMap() const;
};

template <typename T>
const std::string PlannerBase<T>::kWorldFrame = std::string("world");

template <typename T>
const std::string PlannerBase<T>::kSaveTwbNm = std::string("stamped_Twb.txt");

template <typename T>
const std::string PlannerBase<T>::kSaveTwcNm = std::string("stamped_Twc.txt");

template <typename T>
const std::string
    PlannerBase<T>::kSaveTwcUENm = std::string("stamped_Twc_ue.txt");

template <typename T>
PlannerBase<T>::PlannerBase(const ros::NodeHandle& nh,
                            const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  set_planner_state_srv_ = this->pnh_.advertiseService(
      "set_planner_state", &PlannerBase::setPlannerStateCallback, this);
  plan_vis_save_srv_ = this->pnh_.advertiseService(
      "plan_vis_save", &PlannerBase::planVisSaveCallback, this);
  reset_planner_srv_ = this->pnh_.advertiseService(
      "reset_planner", &PlannerBase::resetPlannerCallback, this);
  pub_vis_srv_ = this->pnh_.advertiseService(
      "publish_traj_vis", &PlannerBase::publishVisualizationCallback, this);

  esdf_server_.reset(new voxblox::EsdfServer(nh_, pnh_));
  act_map_server_.reset(new act_map_ros::ActMapServer<T>(nh_, pnh_));

  update_vis_srv_ = pnh_.advertiseService(
      "update_all_vis", &PlannerBase::updateAllVisualizationCallback, this);
  clear_map_srv = pnh_.advertiseService(
      "clear_all_maps", &PlannerBase::clearAllMapCallback, this);

  traj_orient_pub_ =
      pnh_.advertise<visualization_msgs::MarkerArray>("traj_orient", 50);
  traj_pos_pub_ = pnh_.advertise<PointCloud>("traj_pos", 50);
  general_marker_pub_ =
      pnh_.advertise<visualization_msgs::Marker>("general_markers", 10);

  const bool info_metric_use_log_det =
      rpg_ros::param<bool>(this->pnh_, "info_metric_use_log_det", true);
  act_map::kInfoMetricUseLogDet = info_metric_use_log_det;
  createInfoPotential();
}

template <typename T>
void PlannerBase<T>::createInfoPotential()
{
  act_map::InfoPotentialOptions ip_options =
      act_map_ros::readInfoPotentialOptions(this->pnh_);
  VLOG(1) << "Creating potential function for information...";
  info_pot_ptr_.reset(new act_map::InformationPotential<T>(
      ip_options,
      this->act_map_server_->getActMapCRef().options_.vis_options_.at(0)));
  VLOG(1) << (*info_pot_ptr_);
}

template <typename T>
bool PlannerBase<T>::updateAllVisualizationCallback(
    std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  esdf_server_->publishPointclouds();
  act_map_server_->updateVisualization();
  return true;
}

template <typename T>
bool PlannerBase<T>::clearAllMapCallback(std_srvs::Empty::Request& req,
                                         std_srvs::Empty::Response& res)
{
  esdf_server_->clear();
  act_map_server_->getActMapRef().clearMap();
  return true;
}

template <typename T>
bool PlannerBase<T>::hasValidESDFMap() const
{
  return esdf_server_->getEsdfMapPtr()
             ->getEsdfLayerPtr()
             ->getNumberOfAllocatedBlocks() != 0;
}

template <typename T>
bool PlannerBase<T>::hasValidInfoMap() const
{
  return act_map_server_->getActMapCRef()
             .kerLayerCRef()
             .getNumberOfAllocatedBlocks() != 0;
}

template <typename T>
bool PlannerBase<T>::resetPlannerCallback(std_srvs::Empty::Request& /*req*/,
                                          std_srvs::Empty::Response& /*res*/)
{
  resetPlannerState();
  return true;
}

template <typename T>
bool PlannerBase<T>::publishVisualizationCallback(
    std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
{
  visualize();
  return true;
}

template <typename T>
bool PlannerBase<T>::planVisSaveCallback(std_srvs::Empty::Request& /*req*/,
                                         std_srvs::Empty::Response& /*res*/)
{
  plan();
  visualize();
  saveResults();
  return true;
}

}  // namespace act_map_exp
