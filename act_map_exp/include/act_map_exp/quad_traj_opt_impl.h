#pragma once

#include "act_map_exp/quad_traj_opt.h"

#include <chrono>

#include <rpg_common_ros/params_helper.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>
#include <vi_utils/common_utils.h>
#include <act_map_ros/params_reader.h>
#include <unrealcv_bridge/ue_utils.hpp>

#include "act_map_exp/viz_utils.h"

namespace act_map_exp
{
template <typename T>
QuadTrajOpt<T>::QuadTrajOpt(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : PlannerBase<T>(nh, pnh), pos_poly_opt_(3), yaw_poly_opt_(1)
{
  readROSParams();
  resetPlannerState();

  traj_orient_pub_stageone_ = this->pnh_.template advertise<visualization_msgs::MarkerArray>("traj_"
                                                                                             "orien"
                                                                                             "t_"
                                                                                             "inter"
                                                                                             "media"
                                                                                             "te",
                                                                                             50);
  traj_pos_pub_stageone_ = this->pnh_.template advertise<PointCloud>("traj_pos_intermediate", 5);
  failed_sample_points_pub_ = this->pnh_.template advertise<PointCloud>("failed_sample_points", 5);
  pot_.reset(new PotentialCost(options_.robot_radius_, options_.col_margin_));

  // play traj
  play_traj_pos_pub_ = this->pnh_.template advertise<PointCloud>("play_traj_pos", 5);
  play_traj_orient_pub_ =
      this->pnh_.template advertise<visualization_msgs::MarkerArray>("play_traj_orient", 50);
  play_traj_viz_link_pub_ =
      this->pnh_.template advertise<visualization_msgs::Marker>("play_traj_viz_link", 50);
  play_traj_srv_ =
      this->pnh_.advertiseService("play_planned_traj", &QuadTrajOpt::playPlannedTrajCallback, this);

  std::cout << "===========================\n";
  std::cout << "=== Initialization Done ===\n";
  std::cout << "===========================\n";
}

template <typename T>
void QuadTrajOpt<T>::readROSParams()
{
  options_.integral_cost_sample_dt_ =
      rpg_ros::param<double>(this->pnh_, "integral_cost_sample_dt", 0.1);
  CHECK_GT(options_.integral_cost_sample_dt_, 0.0);
  options_.robot_radius_ = rpg_ros::param<double>(this->pnh_, "robot_radius", 0.5);
  CHECK_GT(options_.robot_radius_, 0.0);
  options_.col_margin_ = rpg_ros::param<double>(this->pnh_, "collision_margin", 0.5);
  CHECK_GT(options_.col_margin_, 0.0);

  options_.waypoint_zero_deriv_max_ = rpg_ros::param<int>(this->pnh_, "wp_zero_deriv_max", 0);

  options_.viz_every_iter_pause_sec_ =
      rpg_ros::param<double>(this->pnh_, "viz_every_iter_pause_sec", 1.0);

  options_.viz_observations_every_n_ =
      rpg_ros::param<int>(this->pnh_, "viz_observations_every_n", 1);

  std::string info_type_str =
      rpg_ros::param<std::string>(this->pnh_, "info_metric_type", std::string("trace"));
  CHECK(act_map::kNameToInfoMetric.find(info_type_str) != act_map::kNameToInfoMetric.end());
  options_.info_metric_type_ = act_map::kNameToInfoMetric.at(info_type_str);

  options_.ceres_parameter_tolerance_ =
      rpg_ros::param<double>(this->pnh_, "ceres_parameter_tolerance", 1e-8);
  options_.ceres_function_tolerance_ =
      rpg_ros::param<double>(this->pnh_, "ceres_function_tolerance", 1e-6);
  const std::string ceres_line_search_str =
      rpg_ros::param<std::string>(this->pnh_, "ceres_line_search_type", std::string("LBFGS"));
  CHECK(kStrToCeresLSType.find(ceres_line_search_str) != kStrToCeresLSType.end());
  options_.ceres_line_search_type_ = kStrToCeresLSType.at(ceres_line_search_str);
  options_.ceres_max_iter_ = rpg_ros::param<int>(this->pnh_, "ceres_max_iter", 50);

  options_.second_stage_opt_yaw_only_ =
      rpg_ros::param<bool>(this->pnh_, "second_stage_opt_yaw_only", false);

  options_.ceres_use_logger_ = rpg_ros::param<bool>(this->pnh_, "ceres_use_logger", false);

  options_.visualize_visible_points_ =
      rpg_ros::param<bool>(this->pnh_, "visualize_visible_points", false);

  options_.visualize_iter_cnt_ = rpg_ros::param<bool>(this->pnh_, "visualize_iter_cnt", false);
  options_.text_scale_ = rpg_ros::param<double>(this->pnh_, "visualize_text_scale", 5.0);

  options_.save_sample_dt_ = rpg_ros::param<double>(this->pnh_, "save_sample_dt", 0.1);
}

template <typename T>
void QuadTrajOpt<T>::loadFromYaml(const std::string& param_fn)
{
  VLOG(1) << "Loading parameters from " << param_fn;
  YAML::Node node = YAML::LoadFile(param_fn);
  CHECK(node.IsMap());
  // base parameters
  if (node["integral_cost_sample_dt"])
  {
    options_.integral_cost_sample_dt_ = node["integral_cost_sample_dt"].as<double>();
    VLOG(1) << "- set integral dt to " << options_.integral_cost_sample_dt_;
  }

  if (node["robot_radius"])
  {
    options_.robot_radius_ = node["robot_radius"].as<double>();
    VLOG(1) << "- set robot radius to " << options_.robot_radius_;
  }

  if (node["collision_margin"])
  {
    options_.col_margin_ = node["collision_margin"].as<double>();
    VLOG(1) << "- set collision margin to " << options_.col_margin_;
  }

  if (node["waypoint_zero_deriv_max"])
  {
    options_.waypoint_zero_deriv_max_ = node["waypoint_zero_deriv_max"].as<int>();
    VLOG(1) << "- set max zero deriv. to " << options_.waypoint_zero_deriv_max_;
  }

  if (node["viz_every_iter_pause_sec"])
  {
    options_.viz_every_iter_pause_sec_ = node["viz_every_iter_pause_sec"].as<double>();
    VLOG(1) << "- visualize every iteration pause " << options_.viz_every_iter_pause_sec_;
  }

  if (node["viz_observations_every_n"])
  {
    options_.viz_observations_every_n_ = node["viz_observations_every_n"].as<double>();
    VLOG(1) << "- visualize every n observations:" << options_.viz_observations_every_n_;
  }

  if (node["viz_link_size"])
  {
    options_.viz_link_size_ = node["viz_link_size"].as<double>();
    VLOG(1) << "- visualization link size:" << options_.viz_link_size_;
  }

  if (node["info_metric_type"])
  {
    const std::string info_t_str = node["info_metric_type"].as<std::string>();
    options_.info_metric_type_ = act_map::kNameToInfoMetric.at(info_t_str);
    VLOG(1) << "- set information metric type to " << info_t_str;
  }

  if (node["ceres_parameter_tolerance"])
  {
    options_.ceres_parameter_tolerance_ = node["ceres_parameter_tolerance"].as<double>();
    VLOG(1) << "- set ceres parameter tolerance to " << options_.ceres_parameter_tolerance_;
  }

  if (node["ceres_function_tolerance"])
  {
    options_.ceres_function_tolerance_ = node["ceres_function_tolerance"].as<double>();
    VLOG(1) << "- set ceres function tolerance to " << options_.ceres_function_tolerance_;
  }

  if (node["ceres_max_iter"])
  {
    options_.ceres_max_iter_ = node["ceres_max_iter"].as<int>();
    VLOG(1) << "- set maximum iteration in ceres to: " << options_.ceres_max_iter_;
  }

  if (node["ceres_use_logger"])
  {
    options_.ceres_use_logger_ = node["ceres_use_logger"].as<bool>();
    VLOG(1) << "- set ceres to use logger: " << asStr(options_.ceres_use_logger_);
  }

  if (node["second_stage_opt_yaw_only"])
  {
    options_.second_stage_opt_yaw_only_ = node["second_stage_opt_yaw_only"].as<bool>();
    VLOG(1) << "second stage use yaw only: " << asStr(options_.second_stage_opt_yaw_only_);
  }

  if (node["ceres_line_search_type"])
  {
    const std::string ceres_ls_type_str = node["ceres_line_search_type"].as<std::string>();
    CHECK(kStrToCeresLSType.find(ceres_ls_type_str) != kStrToCeresLSType.end());
    VLOG(1) << "Line search type is set to " << ceres_ls_type_str;
    options_.ceres_line_search_type_ = kStrToCeresLSType.at(ceres_ls_type_str);
  }

  if (node["info_metric_use_log_det"])
  {
    const bool use_log_det = node["info_metric_use_log_det"].as<bool>();
    if (act_map::kInfoMetricUseLogDet != use_log_det)
    {
      act_map::kInfoMetricUseLogDet = options_.info_metric_use_log_det_;
      this->createInfoPotential();
    }
    VLOG(1) << "Reinitialize info. metric potential function.";
  }

  // visualization
  if (node["text_pos"])
  {
    const YAML::Node& text_pos = node["text_pos"];
    CHECK_EQ(text_pos.size(), 3u);

    for (size_t i = 0; i < text_pos.size(); i++)
    {
      text_pos_[i] = text_pos[i].as<double>();
    }

    VLOG(1) << "Show text at: " << text_pos_.transpose();
  }
  if (node["visualize_iter_cnt"])
  {
    options_.visualize_iter_cnt_ = node["visualize_iter_cnt"].as<bool>();
    VLOG(1) << "Visualize iterations: " << asStr(options_.visualize_iter_cnt_);
  }
  if (node["visualize_text_scale"])
  {
    options_.text_scale_ = node["visualize_text_scale"].as<double>();
    VLOG(1) << "Text scale: " << options_.text_scale_;
  }

  // trajectory specific
  CHECK(node["total_time"]);
  total_time_sec_ = node["total_time"].as<double>();
  VLOG(1) << "- total time for trajectory is " << total_time_sec_;

  CHECK(node["waypoints"]);
  pos_wps_.clear();
  yaw_wps_.clear();
  VLOG(1) << "- read waypoints";
  for (const YAML::Node& wp : node["waypoints"])
  {
    constexpr double kDegToRad = M_PI / 180.0;
    CHECK(wp.IsSequence());
    CHECK_EQ(wp.size(), 4u);
    pos_wps_.emplace_back(
        Eigen::Vector3d(wp[0].as<double>(), wp[1].as<double>(), wp[2].as<double>()));
    yaw_wps_.emplace_back(wp[3].as<double>() * kDegToRad);
    VLOG(1) << "--- pos: " << pos_wps_.back().transpose() << "; yaw: " << yaw_wps_.back();
  }

  CHECK(node["num_segments"]);
  num_segments_.clear();
  for (const YAML::Node& seg : node["num_segments"])
  {
    num_segments_.emplace_back(seg.as<int>());
    VLOG(1) << "--- # segments: " << num_segments_.back();
    CHECK_GT(num_segments_.back(), 0);
  }
  CHECK_EQ(num_segments_.size() + 1u, pos_wps_.size());

  // optimization type
  CHECK(node["opt_type"]);
  const std::string traj_opt_str = node["opt_type"].as<std::string>();
  VLOG(1) << "- optimization type is " << traj_opt_str;
  CHECK(kStrToTrajOptT.find(traj_opt_str) != kStrToTrajOptT.end());
  traj_opt_type_ = kStrToTrajOptT.at(traj_opt_str);

  CHECK(node["T_BC"]);
  const YAML::Node& T_ele = node["T_BC"];
  CHECK_EQ(T_ele.size(), 16u);
  rpg::Pose::TransformationMatrix Tbc_mat;
  for (size_t i = 0; i < T_ele.size(); i++)
  {
    Tbc_mat(i / 4, i % 4) = T_ele[i].as<double>();
  }
  VLOG(1) << "- Tbc is\n" << Tbc_mat;
  Tbc_ = rpg::Pose(Tbc_mat);

  if (traj_opt_type_ == TrajOptType::kCeres)
  {
    CHECK(node["ceres_cost"]);
    ceres_cost_to_weight_map_.clear();
    for (size_t ci = 0; ci < node["ceres_cost"].size(); ci++)
    {
      for (const auto& cost_pair : node["ceres_cost"][ci])
      {
        const std::string cost_nm = cost_pair.first.as<std::string>();
        CHECK(kStrToTrajCeresCostT.find(cost_nm) != kStrToTrajCeresCostT.end());
        const double weight = cost_pair.second.as<double>();
        VLOG(1) << "Adding cost " << cost_nm << " with weight " << weight;
        ceres_cost_to_weight_map_.insert({ kStrToTrajCeresCostT.at(cost_nm), weight });
        // some information
        if (cost_nm == "ESDF" && !this->hasValidESDFMap())
        {
          LOG(WARNING) << "No valid esdf map. ESDF cost term will not take "
                          "effect.";
        }
        if (cost_nm == "Info" && !this->hasValidInfoMap())
        {
          if (!this->hasValidInfoMap())
          {
            LOG(WARNING) << "No valid info map. Info cost term will not take "
                            "effect.";
          }
          if (options_.info_metric_type_ == act_map::InfoMetricType::kNone)
          {
            LOG(WARNING) << "Info. metric to None, no effect.";
          }
        }
      }
    }
    CHECK(node["ceres_opt_type"]);
    const std::string ceres_opt_type_str = node["ceres_opt_type"].as<std::string>();
    CHECK(kStrToTrajCeresOptType.find(ceres_opt_type_str) != kStrToTrajCeresOptType.end());
    ceres_opt_type_ = kStrToTrajCeresOptType.at(ceres_opt_type_str);
    VLOG(1) << "- Ceres optimization is set to " << ceres_opt_type_str;

    CHECK(node["calculate_info_from_pc"]);
    calculate_info_from_pc_ = node["calculate_info_from_pc"].as<bool>();
    VLOG(1) << "Will calculate info. metric from point cloud: " << asStr(calculate_info_from_pc_);

    if (node["save_traj_abs_dir"])
    {
      save_traj_abs_dir_ = node["save_traj_abs_dir"].as<std::string>();
      CHECK(rpg::fs::pathExists(save_traj_abs_dir_)) << save_traj_abs_dir_ << " does not exist.";
    }
    else
    {
      save_traj_abs_dir_.clear();
    }
    VLOG(1) << "Will save trajectory in: " << asStr(save_traj_abs_dir_);
  }
}

template <typename T>
void QuadTrajOpt<T>::resetPlannerState()
{
  traj_opt_type_ = TrajOptType::kUnknown;
  pos_wps_.clear();
  yaw_wps_.clear();
  has_valid_traj_ = false;

  ceres_cost_to_weight_map_.clear();
  total_time_sec_ = -1.0;
  num_segments_.clear();
  Tbc_.setIdentity();

  ceres_param_type_ = CeresParamType::kNone;

  ceres_last_raw_costs_.clear();
  ceres_last_integral_cost_samples_.clear();
  integral_points_.clear();
  failed_sample_points_.clear();

  logger_total_cost_history_.clear();
  logger_weighted_cost_history_.clear();
}

template <typename T>
void QuadTrajOpt<T>::plan()
{
  if (traj_opt_type_ == TrajOptType::kUnknown)
  {
    LOG(WARNING) << "Unknown planner state: forgot to set?";
    return;
  }
  else if (traj_opt_type_ == TrajOptType::kLinear)
  {
    VLOG(1) << ">>> start linear trajectory optimization...";
    planLinear();
    has_valid_traj_ = true;
  }
  else if (traj_opt_type_ == TrajOptType::kCeres)
  {
    VLOG(1) << ">>> start trajectory optimization using Ceres...";
    planCeres();
    has_valid_traj_ = true;
  }
}

template <typename T>
void QuadTrajOpt<T>::setUpPolyTraj()
{
  const size_t total_seg = std::accumulate(num_segments_.begin(), num_segments_.end(), 0);
  const std::vector<double> times(total_seg, total_time_sec_ / total_seg);
  VLOG(1) << "Total segment is " << total_seg;
  mtg::Vertex::Vector pos_vertices(total_seg + 1, mtg::Vertex(3));
  mtg::Vertex::Vector yaw_vertices(total_seg + 1, mtg::Vertex(1));
  size_t next_vert_idx = 0;
  for (size_t wpi = 0; wpi < pos_wps_.size(); wpi++)
  {
    VLOG(1) << " => " << wpi << "th waypoint:";
    if (wpi == 0 || (wpi == pos_wps_.size() - 1))
    {
      VLOG(1) << "- mark start or end";
      pos_vertices[next_vert_idx].makeStartOrEnd(pos_wps_[wpi], kPosDerivOptim);
      yaw_vertices[next_vert_idx].makeStartOrEnd(rpg::Matrix11(yaw_wps_[wpi]), kYawDerivOptim);
    }
    else
    {
      VLOG(1) << "- set " << next_vert_idx << "th vertice.";
      pos_vertices[next_vert_idx].addConstraint(0, pos_wps_[wpi]);
      yaw_vertices[next_vert_idx].addConstraint(0, rpg::Matrix11(yaw_wps_[wpi]));
      for (int i = 1; i <= options_.waypoint_zero_deriv_max_; i++)
      {
        pos_vertices[next_vert_idx].addConstraint(i, 0.0);
        yaw_vertices[next_vert_idx].addConstraint(i, 0.0);
      }
    }
    next_vert_idx += num_segments_[wpi];
  }
  pos_poly_opt_.setupFromVertices(pos_vertices, times, kPosDerivOptim);
  yaw_poly_opt_.setupFromVertices(yaw_vertices, times, kYawDerivOptim);

  pth_.setPolyTraj(&pos_poly_opt_);
  yth_.setPolyTraj(&yaw_poly_opt_);
}

template <typename T>
void QuadTrajOpt<T>::planLinear()
{
  rpg::Timer timer;
  timer.start();

  // position
  {
    pos_poly_opt_.solveLinear();
  }

  // yaw
  {
    yaw_poly_opt_.solveLinear();
  }

  VLOG(1) << "Linear planning took (sec) " << timer.stop();
}

template <typename T>
void QuadTrajOpt<T>::planCeres()
{
  // linear solver to get initial values
  planLinear();

  last_plan_time_ceres_solve_ = 0.0;
  last_plan_time_logger_ = 0.0;

  // get the parameters
  PolyParams pos_free_constraints, yaw_free_constraints;
  this->getFreeConstraints(&pos_free_constraints, &yaw_free_constraints);
  CHECK_EQ(pos_free_constraints.size(), 3u);
  CHECK_EQ(yaw_free_constraints.size(), 1u);
  CHECK_EQ(yth_.n_free_, static_cast<size_t>(yaw_free_constraints.front().size()));

  if (ceres_opt_type_ != TrajCeresOptType::kTwoStages)
  {
    // one shot optimization
    if (ceres_opt_type_ == TrajCeresOptType::kPosition)
    {
      ceres_param_type_ = CeresParamType::kPosition;
    }
    else if (ceres_opt_type_ == TrajCeresOptType::kYaw)
    {
      ceres_param_type_ = CeresParamType::kYaw;
    }
    else if (ceres_opt_type_ == TrajCeresOptType::kJoint)
    {
      ceres_param_type_ = CeresParamType::kPositionYaw;
    }
    last_plan_time_ceres_solve_ += ceresSolve(&pos_free_constraints, &yaw_free_constraints);
  }
  else if (ceres_opt_type_ == TrajCeresOptType::kTwoStages)
  {
    // two passes
    VLOG(1) << "1st pass: optimize position w/o considering info.";
    ceres_param_type_ = CeresParamType::kPosition;
    auto it = ceres_cost_to_weight_map_.find(TrajCeresCostType::kInfo);
    if (it != ceres_cost_to_weight_map_.end())
    {
      const double inf_w = it->second;
      VLOG(1) << "Cache information weight " << inf_w;
      ceres_cost_to_weight_map_.erase(it);
      last_plan_time_ceres_solve_ += ceresSolve(&pos_free_constraints, &yaw_free_constraints);
      CHECK(ceres_cost_to_weight_map_.find(TrajCeresCostType::kInfo) ==
            ceres_cost_to_weight_map_.end());
      ceres_cost_to_weight_map_.insert({ TrajCeresCostType::kInfo, inf_w });
      for (const auto& kv : ceres_cost_to_weight_map_)
      {
        VLOG(1) << "restored: " << kTrajCeresCostTToStr.at(kv.first) << ": " << kv.second;
      }
    }
    else
    {
      last_plan_time_ceres_solve_ += ceresSolve(&pos_free_constraints, &yaw_free_constraints);
    }

    if (traj_pos_pub_stageone_.getNumSubscribers() != 0 ||
        traj_orient_pub_stageone_.getNumSubscribers() != 0)
    {
      sampleAndVisualizeCurrentTraj(traj_pos_pub_stageone_, traj_orient_pub_stageone_,
                                    Eigen::Vector3d(1, 0, 0), "intermediate");
    }

    VLOG(1) << "2nd pass: optimize yaw";
    if (options_.second_stage_opt_yaw_only_)
    {
      ceres_param_type_ = CeresParamType::kYaw;
    }
    else
    {
      ceres_param_type_ = CeresParamType::kPositionYaw;
    }
    last_plan_time_ceres_solve_ += ceresSolve(&pos_free_constraints, &yaw_free_constraints);
  }
  else
  {
    LOG(FATAL) << "unknown ceres optimization type.";
  }

  // set back to the polynomial
  this->setFreeConstraints(pos_free_constraints, yaw_free_constraints);
}

template <typename T>
double QuadTrajOpt<T>::ceresSolve(PolyParams* pos_free_constraints,
                                  PolyParams* yaw_free_constraints)
{
  if (ceres_param_type_ == CeresParamType::kNone)
  {
    LOG(WARNING) << "Not putting anything in Ceres, no optimization.";
    return 0.0;
  }

  initCeresCostCacheLastIter();

  VLOG(10) << "==== initial parameters:";
  for (size_t i = 0; i < pos_free_constraints->size(); i++)
  {
    VLOG(10) << (*pos_free_constraints)[i].transpose();
  }
  VLOG(10) << (*yaw_free_constraints)[0].transpose();

  // pack
  std::vector<double> params;

  if (ceresHasPosition())
  {
    packVecVecXd(*pos_free_constraints, &params);
  }
  if (ceresHasYaw())
  {
    packVecVecXd(*yaw_free_constraints, &params);
  }
  CHECK(params.size() == (3 * pth_.n_free_ + yth_.n_free_) || params.size() == 3 * pth_.n_free_ ||
        params.size() == yth_.n_free_);

  // set up ceres and solve
  ceres::GradientProblem problem(new PlanCeresFunction<T>(this));
  ceres::GradientProblemSolver::Options ceres_opts;
  ceres_opts.minimizer_progress_to_stdout = false;
  ceres_opts.parameter_tolerance = options_.ceres_parameter_tolerance_;
  ceres_opts.function_tolerance = options_.ceres_function_tolerance_;
  ceres_opts.line_search_direction_type = options_.ceres_line_search_type_;
  ceres_opts.max_num_iterations = options_.ceres_max_iter_;
  LogTrajOpt<T> logger(this, params);
  if (options_.ceres_use_logger_)
  {
    ceres_opts.update_state_every_iteration = true;
    ceres_opts.callbacks.push_back(&logger);
  }
  //  ceres_opts.line_search_direction_type = ceres::BFGS;
  //  ceres_opts.line_search_interpolation_type = ceres::BISECTION;
  rpg::Timer timer;
  timer.start();
  ceres::Solve(ceres_opts, problem, params.data(), &last_opt_summary_);
  const double ceres_solve_time = timer.stop();
  std::cout << last_opt_summary_.FullReport() << std::endl;

  if (options_.ceres_use_logger_)
  {
    std::cout << logger.summaryAsStr();
  }

  // unpack
  unpackCeresParams(params.data(), false, pos_free_constraints, yaw_free_constraints);
  return ceres_solve_time;
}

template <typename T>
void QuadTrajOpt<T>::initCeresCostCacheLastIter() const
{
  ceres_last_raw_costs_.clear();
  ceres_last_integral_cost_samples_.clear();
  for (const auto& kv : ceres_cost_to_weight_map_)
  {
    ceres_last_raw_costs_.insert({ kv.first, 0.0 });
    if (isIntegralType(kv.first))
    {
      ceres_last_integral_cost_samples_.insert({ kv.first, TimeValues() });
    }
  }
}

template <typename T>
double QuadTrajOpt<T>::calculateCostAndGrad(PolyParams* pos_grad, PolyParams* yaw_grad) const
{
  if (pos_grad == nullptr && yaw_grad == nullptr)
  {
    VLOG(5) << "Not calculating gradient, just the cost.";
  }
  if (pos_grad)
  {
    setZero(pos_grad);
  }
  if (yaw_grad)
  {
    setZero(yaw_grad);
  }
  double accumulated_cost = 0.0;

  VLOG(10) << "=== Calculating cost and grad ===";
  std::vector<TrajCeresCostType> integ_types;
  std::vector<double> integ_type_weights;
  for (const auto& cost_pair : ceres_cost_to_weight_map_)
  {
    if (cost_pair.first == TrajCeresCostType::kPosDynamic)
    {
      PolyParams pos_grad_i;
      std::tie(pos_grad_i, std::ignore) = getVectorsOfPolyDerivSize();
      PolyParams* pos_grad_i_ptr = pos_grad != nullptr ? &pos_grad_i : nullptr;

      const double dyn_cost = pth_.getDynamicCost(pos_grad_i_ptr);
      ceres_last_raw_costs_[TrajCeresCostType::kPosDynamic] = dyn_cost;

      const double w_dyn_cost = cost_pair.second * dyn_cost;
      accumulated_cost += w_dyn_cost;

      VLOG(10) << "- Position dynamic cost: " << w_dyn_cost << " (" << dyn_cost << " with weight "
               << cost_pair.second << ")";
      if (pos_grad_i_ptr)
      {
        accumulate((*pos_grad_i_ptr), cost_pair.second, pos_grad);
        VLOG(10) << "- Dynamic position grad (w/o weight): "
                 << "\n -" << (*pos_grad_i_ptr)[0].transpose() << "\n -"
                 << (*pos_grad_i_ptr)[1].transpose() << "\n -" << (*pos_grad_i_ptr)[2].transpose();
      }
    }
    else if (cost_pair.first == TrajCeresCostType::kYawDynamic)
    {
      PolyParams yaw_grad_i;
      std::tie(std::ignore, yaw_grad_i) = getVectorsOfPolyDerivSize();
      PolyParams* yaw_grad_i_ptr = yaw_grad != nullptr ? &yaw_grad_i : nullptr;

      const double dyn_cost = yth_.getDynamicCost(yaw_grad_i_ptr);
      ceres_last_raw_costs_[TrajCeresCostType::kYawDynamic] = dyn_cost;

      const double w_dyn_cost = cost_pair.second * dyn_cost;
      accumulated_cost += w_dyn_cost;

      VLOG(10) << "- Yaw dynamic cost: " << w_dyn_cost << " (" << dyn_cost << " with weight "
               << cost_pair.second << ")";
      if (yaw_grad_i_ptr)
      {
        accumulate((*yaw_grad_i_ptr), cost_pair.second, yaw_grad);
        VLOG(10) << "- Dynamic yaw grad (w/o weight): "
                 << "\n -" << (*yaw_grad_i_ptr)[0].transpose() << "\n -"
                 << (*yaw_grad_i_ptr)[1].transpose() << "\n -" << (*yaw_grad_i_ptr)[2].transpose();
      }
    }
    else if (cost_pair.first == TrajCeresCostType::kESDF)
    {
      if (!this->hasValidESDFMap())
      {
        continue;
      }
      integ_types.emplace_back(cost_pair.first);
      integ_type_weights.emplace_back(cost_pair.second);
    }
    else if (cost_pair.first == TrajCeresCostType::kInfo)
    {
      CHECK(this->info_pot_ptr_);
      if (!this->hasValidInfoMap() || options_.info_metric_type_ == act_map::InfoMetricType::kNone)
      {
        continue;
      }
      integ_types.emplace_back(cost_pair.first);
      integ_type_weights.emplace_back(cost_pair.second);
    }
  }

  if (integ_types.size() > 0)
  {
    // calculate integral type costs
    std::vector<PolyParams> inte_pos_grad(integ_types.size(),
                                          PolyParams(3, Eigen::VectorXd(pth_.n_free_)));
    std::vector<PolyParams> inte_yaw_grad(integ_types.size(),
                                          PolyParams(1, Eigen::VectorXd(yth_.n_free_)));
    std::vector<PolyParams>* inte_pos_grad_ptr = pos_grad != nullptr ? &inte_pos_grad : nullptr;
    std::vector<PolyParams>* inte_yaw_grad_ptr = yaw_grad != nullptr ? &inte_yaw_grad : nullptr;

    std::vector<double> integral_cost(integ_types.size(), 0.0);
    calculateIntegralCost(integ_types, &integral_cost, inte_pos_grad_ptr, inte_yaw_grad_ptr);

    for (size_t i = 0; i < integ_types.size(); i++)
    {
      const double cur_cost = integral_cost[i];
      const double& w = integ_type_weights[i];
      const double cur_w_cost = w * cur_cost;
      accumulated_cost += cur_w_cost;
      VLOG(10) << "- Integral cost: " << kTrajCeresCostTToStr.at(integ_types[i]) << " "
               << cur_w_cost << " (" << cur_cost << " with weight " << w << ")";
      ceres_last_raw_costs_[integ_types[i]] = cur_cost;
      if (inte_pos_grad_ptr)
      {
        accumulate((*inte_pos_grad_ptr)[i], w, pos_grad);
        VLOG(10) << "- Integral position grad (w/o weight): "
                 << kTrajCeresCostTToStr.at(integ_types[i]) << "\n -"
                 << (*inte_pos_grad_ptr)[i][0].transpose() << "\n -"
                 << (*inte_pos_grad_ptr)[i][1].transpose() << "\n -"
                 << (*inte_pos_grad_ptr)[i][2].transpose();
      }
      if (inte_yaw_grad_ptr)
      {
        accumulate((*inte_yaw_grad_ptr)[i], w, yaw_grad);
        VLOG(10) << "- Integral yaw grad (w/o weight): " << kTrajCeresCostTToStr.at(integ_types[i])
                 << "\n -" << (*inte_yaw_grad_ptr)[i][0].transpose();
      }
    }
  }

  return accumulated_cost;
}

template <typename T>
void QuadTrajOpt<T>::calculateIntegralCost(const std::vector<TrajCeresCostType>& types,
                                           std::vector<double>* costs,
                                           std::vector<PolyParams>* pos_grad,
                                           std::vector<PolyParams>* yaw_grad) const
{
  for (const TrajCeresCostType& c : types)
  {
    VLOG(10) << "Calculate for error " << kTrajCeresCostTToStr.at(c);
  }
  CHECK_NOTNULL(costs);
  CHECK_EQ(types.size(), costs->size());
  CHECK(ceres_param_type_ != CeresParamType::kNone);
  if (pos_grad)
  {
    for (PolyParams& p : *pos_grad)
    {
      setZero(&p);
    }
    CHECK_EQ(types.size(), pos_grad->size());
  }
  if (yaw_grad)
  {
    for (PolyParams& p : *yaw_grad)
    {
      setZero(&p);
    }
    CHECK_EQ(types.size(), yaw_grad->size());
  }
  integral_points_.clear();
  failed_sample_points_.clear();

  std::fill(costs->begin(), costs->end(), 0.0);
  const bool need_acc =
      (std::find(types.begin(), types.end(), TrajCeresCostType::kInfo) != types.end());
  const bool need_yaw =
      (std::find(types.begin(), types.end(), TrajCeresCostType::kInfo) != types.end());

  // position and velocity are needed anyway
  VLOG(10) << "Calculating polynomial coefficients ...";
  pth_.updateCoefs();
  pth_.updateVelCoefs();
  if (need_acc)
  {
    pth_.updateAccCoefs();
  }
  if (need_yaw)
  {
    yth_.updateCoefs();
  }

  std::vector<double> seg_times;
  pos_poly_opt_.getSegmentTimes(&seg_times);
  for (const TrajCeresCostType& ct : types)
  {
    CHECK(ceres_last_integral_cost_samples_.find(ct) != ceres_last_integral_cost_samples_.end());
    ceres_last_integral_cost_samples_.at(ct).clear();
  }
  for (size_t seg_idx = 0; seg_idx < seg_times.size(); seg_idx++)
  {
    VLOG(10) << "- Process segment " << seg_idx << " ....";
    VLOG(10) << "- Segment duration is " << seg_times[seg_idx];
    std::vector<double> sample_times;
    vi_utils::linspace(0 + 1e-3, seg_times[seg_idx] - 1e-3, options_.integral_cost_sample_dt_,
                       &sample_times);
    for (size_t sample_idx = 0; sample_idx < sample_times.size() - 1; sample_idx++)
    {
      const double sample_t = sample_times[sample_idx];
      VLOG(10) << "-- process time " << sample_idx << " in segment " << seg_idx << ": " << sample_t;
      Eigen::RowVectorXd t_vec_pos;
      pth_.getTVec(sample_t, &t_vec_pos);

      VLOG(10) << "-- calculating position and velocity... ";
      Eigen::Vector3d vel;
      Eigen::RowVectorXd dveli_ddp;
      pth_.getVelocity(seg_idx, t_vec_pos, &vel, &dveli_ddp);
      const double vel_norm = vel.norm();
      if (vel_norm < 1e-6)
      {
        VLOG(10) << "skip current time segment " << sample_idx + 1 << " out of "
                 << sample_times.size() << " in (" << seg_idx + 1 << "/" << seg_times.size() << ")"
                 << " due to low velocity.";
        continue;
      }
      Eigen::Vector3d pos;
      Eigen::RowVectorXd dposi_ddp;
      pth_.getPosition(seg_idx, t_vec_pos, &pos, &dposi_ddp);

      const double dt = sample_times[sample_idx + 1] - sample_t;
      integral_points_.push_back(pos);
      integral_points_.push_back(pos + vel * dt);
      for (size_t cidx = 0; cidx < types.size(); cidx++)
      {
        VLOG(10) << "-- caluclating cost " << cidx << ": " << kTrajCeresCostTToStr.at(types[cidx])
                 << "...";
        const TrajCeresCostType ct = types[cidx];
        ceres_last_integral_cost_samples_.at(ct).times.push_back(sample_t);

        if (ct == TrajCeresCostType::kESDF)
        {
          double distance;
          constexpr bool interpolation = true;
          Eigen::Vector3d esdf_grad;
          if (!this->esdf_server_->getEsdfMapPtr()->getDistanceAndGradientAtPosition(
                  pos, interpolation, &distance, &esdf_grad))
          {
            ceres_last_integral_cost_samples_.at(ct).values.push_back(
                std::numeric_limits<double>::quiet_NaN());
            continue;
          }

          double dpotential_ddist, potential_cost;
          potential_cost = (*pot_)(distance, &dpotential_ddist);
          ceres_last_integral_cost_samples_.at(ct).values.push_back(potential_cost);
          const double cur_cost = vel_norm * dt * potential_cost;
          (*costs)[cidx] += cur_cost;
          VLOG(10) << "--- cost: " << cur_cost;
          VLOG(10) << "--- Vel. deriv: " << dveli_ddp;
          VLOG(10) << "--- Pos. deriv: " << dposi_ddp;
          if (pos_grad)
          {
            PolyParams& cur_grad = (*pos_grad)[cidx];
            for (int dimi = 0; dimi < 3; dimi++)
            {
              Eigen::VectorXd& cur_dim_grad = cur_grad[static_cast<size_t>(dimi)];
              const Eigen::RowVectorXd grad_dim_i =
                  (vel_norm * dpotential_ddist * esdf_grad(dimi) * dposi_ddp +
                   vel(dimi) / vel_norm * dveli_ddp * potential_cost) *
                  dt;
              CHECK_EQ(grad_dim_i.cols(), cur_dim_grad.size());
              cur_dim_grad += grad_dim_i.transpose();
              VLOG(10) << "--- grad for dim. " << dimi << ": " << grad_dim_i;
            }
          }
        }
        else if (ct == TrajCeresCostType::kInfo)
        {
          Eigen::Vector3d acc;
          Eigen::RowVectorXd dacci_ddp;
          pth_.getAcceleration(seg_idx, t_vec_pos, &acc, &dacci_ddp);

          Eigen::RowVectorXd t_vec_yaw;
          yth_.getTVec(sample_t, &t_vec_yaw);
          Eigen::Matrix<double, 1, 1> yaw;
          Eigen::RowVectorXd dyaw_ddp;
          yth_.getPosition(seg_idx, t_vec_yaw, &yaw, &dyaw_ddp);

          rpg::Rotation::RotationMatrix Rwb;
          rpg::Matrix93 dRwb_dacc;
          rpg::Matrix91 dRwb_dyaw;
          quadAccYawToRwb(acc, yaw(0, 0), &Rwb, &dRwb_dacc, &dRwb_dyaw);

          rpg::Pose Twb(pos, rpg::Rotation(Rwb));
          rpg::Pose Twc = Twb * Tbc_;
          rpg::Matrix99 dRwc_dRwb;
          squareMatMulJac(Tbc_.getRotationMatrix(), &dRwc_dRwb);
          rpg::Matrix39 dtwc_dRwb;
          matVecMulJac(Tbc_.getPosition(), &dtwc_dRwb);
          rpg::Matrix33 dtwc_dtwb = rpg::Matrix33::Identity();

          rpg::Matrix39 dphi_dRwc;
          dRotGdR(Twc.getRotationMatrix(), &dphi_dRwc);

          // query the map
          double info_metric = 0.0;
          Eigen::Vector3d dinfo_dtwc, dinfo_dphi;
          bool res = false;
          if (calculate_info_from_pc_)
          {
            res = this->act_map_server_->getActMapCRef().getInfoMetricFromPC(
                Twc, options_.info_metric_type_, &info_metric, &dinfo_dtwc, &dinfo_dphi);
          }
          else
          {
            res = this->act_map_server_->getActMapCRef().getInfoMetricAt(
                Twc, options_.info_metric_type_, &info_metric, &dinfo_dtwc, &dinfo_dphi);
          }
          if (!res)
          {
            LOG(WARNING) << "Failed to compute the info. metric, will skip.";
            failed_sample_points_.emplace_back(Twc.getPosition());
            ceres_last_integral_cost_samples_.at(ct).values.push_back(
                std::numeric_limits<double>::quiet_NaN());
            continue;
          }
          const rpg::Matrix19 dinfo_dRwb =
              dinfo_dphi.transpose() * dphi_dRwc * dRwc_dRwb + dinfo_dtwc.transpose() * dtwc_dRwb;
          const rpg::Matrix13 dinfo_dtwb = dinfo_dtwc.transpose() * dtwc_dtwb;

          // cost aggregation and gradient
          double dpot_dinfo = 0.0;
          double pot_cost = 0.0;
          if (calculate_info_from_pc_)
          {
            pot_cost =
                this->info_pot_ptr_->eval(info_metric, options_.info_metric_type_, &dpot_dinfo);
          }
          else
          {
            pot_cost = this->info_pot_ptr_->evalApproxVis(info_metric, options_.info_metric_type_,
                                                          &dpot_dinfo);
          }
          ceres_last_integral_cost_samples_.at(ct).values.push_back(pot_cost);
          double cur_cost = vel_norm * dt * pot_cost;
          (*costs)[cidx] += cur_cost;
          VLOG(10) << "--- info metric: " << info_metric;
          VLOG(10) << "--- pot cost: " << pot_cost;
          VLOG(10) << "--- cost: " << cur_cost;

          if (pos_grad)
          {
            PolyParams& cur_pos_grad = (*pos_grad)[cidx];
            for (int dimi = 0; dimi < 3; dimi++)
            {
              Eigen::VectorXd& cur_dim_grad = cur_pos_grad[static_cast<size_t>(dimi)];
              Eigen::RowVectorXd grad_dim_i;
              grad_dim_i = (vel.norm() * dpot_dinfo *
                                (dinfo_dtwb(dimi) * dposi_ddp +
                                 dinfo_dRwb * dRwb_dacc.col(dimi) * dacci_ddp) +
                            vel(dimi) / vel.norm() * dveli_ddp * pot_cost) *
                           dt;
              CHECK_EQ(grad_dim_i.cols(), cur_dim_grad.size());
              cur_dim_grad += grad_dim_i.transpose();
              VLOG(10) << "--- grad for dim. " << dimi << ": " << grad_dim_i;
            }
          }
          if (yaw_grad)
          {
            Eigen::VectorXd cur_yaw_grad;
            cur_yaw_grad = (vel.norm() * dpot_dinfo * dinfo_dRwb * dRwb_dyaw * dyaw_ddp) * dt;
            CHECK_EQ(cur_yaw_grad.size(), (*yaw_grad)[cidx][0].size());
            (*yaw_grad)[cidx][0] += cur_yaw_grad;
          }
        }
        else
        {
          LOG(FATAL) << "Unknown integral cost type";
        }
      }
    }
  }
}

template <typename T>
void QuadTrajOpt<T>::sampleTrajectory(const double sample_dt,
                                      mav_msgs::EigenTrajectoryPointVector* states,
                                      std::vector<double>* yaws_rad) const
{
  CHECK(states);
  states->clear();
  CHECK(yaws_rad);
  yaws_rad->clear();

  mtg::Trajectory pos_traj;
  pos_poly_opt_.getTrajectory(&pos_traj);
  mtg::sampleWholeTrajectory(pos_traj, sample_dt, states);

  mtg::Trajectory yaw_traj;
  yaw_poly_opt_.getTrajectory(&yaw_traj);
  for (const mav_msgs::EigenTrajectoryPoint& s : *states)
  {
    const double t_sec = s.time_from_start_ns / mtg::kNumNSecPerSec;
    yaws_rad->emplace_back(yaw_traj.evaluate(t_sec, 0)(0));
  }
}

template <typename T>
void QuadTrajOpt<T>::sampleTrajectory(const double sample_t, rpg::PoseVec* Twb_vec,
                                      std::vector<double>* times_sec) const
{
  CHECK(Twb_vec);
  mav_msgs::EigenTrajectoryPointVector states;
  std::vector<double> yaws_rad;
  this->sampleTrajectory(sample_t, &states, &yaws_rad);

  if (times_sec)
  {
    times_sec->clear();
  }

  for (size_t sidx = 0; sidx < states.size(); sidx++)
  {
    Eigen::Matrix3d rot_mat;
    quadAccYawToRwb(states[sidx].acceleration_W, yaws_rad[sidx], &rot_mat, nullptr, nullptr);
    Twb_vec->emplace_back(rpg::Pose(rpg::Rotation(rot_mat), states[sidx].position_W));
    if (times_sec)
    {
      times_sec->push_back(states[sidx].time_from_start_ns / mtg::kNumNSecPerSec);
    }
  }
}

template <typename T>
void QuadTrajOpt<T>::visualize()
{
  if (!has_valid_traj_)
  {
    LOG(WARNING) << "Do not have a valid trajectory, plan first?";
    return;
  }
  if (this->traj_pos_pub_.getNumSubscribers() == 0 &&
      this->traj_orient_pub_.getNumSubscribers() == 0)
  {
    return;
  }

  sampleAndVisualizeCurrentTraj(this->traj_pos_pub_, this->traj_orient_pub_,
                                Eigen::Vector3d(0, 1, 0), "last_iter");

  if (this->general_marker_pub_.getNumSubscribers() > 0 && integral_points_.size() > 0)
  {
    visualization_msgs::Marker line_list;
    line_list.ns = "integral_points";
    line_list.id = 0;
    line_list.header.frame_id = PlannerBase<T>::kWorldFrame;
    line_list.header.stamp = ros::Time::now();
    line_list.pose.orientation.w = 1.0;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.color.r = 1.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    line_list.color.a = 1.0;
    line_list.scale.x = (integral_points_.back() - integral_points_.front()).norm() /
                        (integral_points_.size() / 2.0) * 0.2;
    for (size_t i = 0; i < integral_points_.size(); i += 2)
    {
      geometry_msgs::Point ps, pe;
      ps.x = integral_points_[i](0);
      ps.y = integral_points_[i](1);
      ps.z = integral_points_[i](2);
      pe.x = integral_points_[i + 1](0);
      pe.y = integral_points_[i + 1](1);
      pe.z = integral_points_[i + 1](2);
      line_list.points.push_back(ps);
      line_list.points.push_back(pe);
    }
    this->general_marker_pub_.publish(line_list);
  }

  if (failed_sample_points_pub_.getNumSubscribers() > 0 && failed_sample_points_.size() > 0)
  {
    PointCloud failed_pc;
    pcl_conversions::toPCL(ros::Time::now(), failed_pc.header.stamp);
    failed_pc.header.frame_id = PlannerBase<T>::kWorldFrame;
    failed_pc.reserve(failed_sample_points_.size());
    for (const auto& fpt : failed_sample_points_)
    {
      Point pt;
      pt.x = fpt(0);
      pt.y = fpt(1);
      pt.z = fpt(2);
      pt.r = 255;
      pt.g = 0;
      pt.b = 0;
      failed_pc.push_back(pt);
    }
    failed_sample_points_pub_.publish(failed_pc);
  }
}

template <typename T>
void QuadTrajOpt<T>::sampleAndVisualizeCurrentTraj(const ros::Publisher& traj_pos_pub,
                                                   const ros::Publisher& traj_orient_pub,
                                                   const Eigen::Vector3d& rgb,
                                                   const std::string& marker_ns) const
{
  constexpr double viz_sample_dt = 0.1;

  rpg::PoseVec Twb_vec;
  this->sampleTrajectory(viz_sample_dt, &Twb_vec);
  visualizeTrajectory(Twb_vec, traj_pos_pub, traj_orient_pub, Tbc_, rgb,
                      PlannerBase<T>::kWorldFrame);
  if (!options_.visualize_visible_points_ || this->general_marker_pub_.getNumSubscribers() == 0)
  {
    return;
  }

  const act_map::ActMap<T>& map = this->act_map_server_->getActMapCRef();
  map.cachePointsAndViewDirs(false);
  std::vector<size_t> states_idx;
  sampleIndices(10, Twb_vec.size(), &states_idx);

  visualization_msgs::Marker marker;
  marker.id = 0;
  marker.ns = marker_ns;

  marker.header.frame_id = PlannerBase<T>::kWorldFrame;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = options_.viz_link_size_;
  marker.scale.y = options_.viz_link_size_;
  marker.scale.z = options_.viz_link_size_;
  marker.color.a = 0.5;
  marker.color.r = rgb(0);
  marker.color.g = rgb(1);
  marker.color.b = rgb(2);

  for (const size_t& sidx : states_idx)
  {
    rpg::Pose Twc = Twb_vec[sidx] * Tbc_;

    act_map::VisIdx vis_idx;
    map.visCheckerCRef().getVisibleIdx(Twc, map.cachedPoints(), map.cachedViewDirs(), &vis_idx);
    int cnt = 0;
    for (const size_t& pt_idx : vis_idx)
    {
      cnt++;
      if (cnt % options_.viz_observations_every_n_ != 0)
      {
        continue;
      }
      const Eigen::Vector3d& pw_i = map.cachedPoints().at(pt_idx);
      geometry_msgs::Point point;
      point.x = static_cast<double>(Twc.getPosition()(0));
      point.y = static_cast<double>(Twc.getPosition()(1));
      point.z = static_cast<double>(Twc.getPosition()(2));
      marker.points.push_back(point);
      point.x = static_cast<double>(pw_i(0));
      point.y = static_cast<double>(pw_i(1));
      point.z = static_cast<double>(pw_i(2));
      marker.points.push_back(point);
    }
  }
  VLOG(5) << "Publish " << marker.points.size() / 2 << " visibility links.";
  this->general_marker_pub_.publish(marker);
}

template <typename T>
void QuadTrajOpt<T>::publishText(const std::string& text, const Eigen::Vector3d& rgb,
                                 const Eigen::Vector3d& pos, const std::string& ns) const
{
  if (this->general_marker_pub_.getNumSubscribers() == 0)
  {
    return;
  }

  visualization_msgs::Marker marker;
  marker.id = 0;
  marker.ns = "text";

  marker.header.frame_id = PlannerBase<T>::kWorldFrame;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = pos.x();
  marker.pose.position.y = pos.y();
  marker.pose.position.z = pos.z();
  marker.scale.x = options_.text_scale_;
  marker.scale.y = options_.text_scale_;
  marker.scale.z = options_.text_scale_;
  marker.color.a = 1.0;
  marker.color.r = rgb(0);
  marker.color.g = rgb(1);
  marker.color.b = rgb(2);
  marker.text = text;

  this->general_marker_pub_.publish(marker);
}

template <typename T>
void QuadTrajOpt<T>::saveResults()
{
  if (save_traj_abs_dir_.empty())
  {
    LOG(WARNING) << "No directory for saving provided, skip saving.";
    return;
  }
  VLOG(1) << "Calculating cost again for saving...";
  initCeresCostCacheLastIter();
  this->calculateCostAndGrad(nullptr, nullptr);

  VLOG(1) << "Sampling trajectory for # features and info. ...";
  rpg::PoseVec Twb_vec;
  std::vector<double> sampled_times;
  this->sampleTrajectory(options_.save_sample_dt_, &Twb_vec, &sampled_times);
  rpg::PoseVec Twc_vec(Twb_vec.size());
  for (size_t idx = 0; idx < Twb_vec.size(); idx++)
  {
    Twc_vec[idx] = Twb_vec[idx] * Tbc_;
  }
  VLOG(1) << "- sampled " << Twc_vec.size() << " body and camera poses.";

  std::vector<size_t> n_ftrs(Twb_vec.size());
  act_map::PoseInfoMatVec info_mat(Twb_vec.size());
  const act_map::ActMap<T>& map = this->act_map_server_->getActMapCRef();
  map.cachePointsAndViewDirs(true);
  for (size_t Tidx = 0; Tidx < Twc_vec.size(); Tidx++)
  {
    const rpg::Pose& Twc = Twc_vec[Tidx];
    act_map::VisIdx vis_idx;
    map.visCheckerCRef().getVisibleIdx(Twc, map.cachedPoints(), map.cachedViewDirs(), &vis_idx);
    VLOG(10) << "Pose " << Tidx << " has " << vis_idx.size() << " visible features.";
    n_ftrs[Tidx] = vis_idx.size();
    info_mat[Tidx].setZero();
    for (const size_t pidx : vis_idx)
    {
      act_map::addPoseInfoBearingGlobal(Twc, map.cachedPoints().at(pidx), nullptr,
                                        &(info_mat[Tidx]));
    }
  }
  VLOG(1) << "- Sampled the information of " << Twc_vec.size() << " poses.";

  VLOG(1) << "Saving...";
  VLOG(1) << "- poses ...";
  saveStampedPoses(sampled_times, Twb_vec, save_traj_abs_dir_ + "/" + PlannerBase<T>::kSaveTwbNm);
  saveStampedPoses(sampled_times, Twc_vec, save_traj_abs_dir_ + "/" + PlannerBase<T>::kSaveTwcNm);
  unrealcv_bridge::UEPoseVec Twc_vec_ue(Twc_vec.size());
  for (size_t Tidx = 0; Tidx < Twc_vec.size(); Tidx++)
  {
    unrealcv_bridge::TwcToUEPose(Twc_vec[Tidx], &(Twc_vec_ue[Tidx]));
  }
  saveStampedPoses(sampled_times, Twc_vec_ue,
                   save_traj_abs_dir_ + "/" + PlannerBase<T>::kSaveTwcUENm);
  VLOG(1) << "- info ...";
  saveStampedEigenMatrices(sampled_times, info_mat, save_traj_abs_dir_ + "/stamped_info.txt");
  saveStampedScalars(sampled_times, n_ftrs, save_traj_abs_dir_ + "/stamped_n_ftrs.txt");
  VLOG(1) << "- costs ...";
  // last samples
  for (const auto& kv : ceres_last_integral_cost_samples_)
  {
    const std::string save_fn = save_traj_abs_dir_ + "/last_iter_sampled_" +
                                kTrajCeresCostTToStr.at(kv.first) + "_cost.txt";
    kv.second.save(save_fn);
  }
  // history
  logger_total_cost_history_.save(save_traj_abs_dir_ + "/total_ceres_cost_history.txt");
  for (const auto& kv : logger_weighted_cost_history_)
  {
    const std::string save_fn =
        save_traj_abs_dir_ + "/weighted_" + kTrajCeresCostTToStr.at(kv.first) + "_cost_history.txt";
    kv.second.save(save_fn);
  }

  YAML::Node last_raw_cost_node;
  for (const auto& kv : ceres_last_raw_costs_)
  {
    last_raw_cost_node[kTrajCeresCostTToStr.at(kv.first)] = kv.second;
  }
  std::ofstream last_raw_fn(save_traj_abs_dir_ + "/raw_costs_last_iter.yaml");
  last_raw_fn << last_raw_cost_node;
  last_raw_fn.close();

  YAML::Node ceres_stats;
  ceres_stats["num_grad_eval"] = last_opt_summary_.num_gradient_evaluations;
  ceres_stats["num_cost_eval"] = last_opt_summary_.num_cost_evaluations;
  ceres_stats["time_cost_eval"] = last_opt_summary_.cost_evaluation_time_in_seconds;
  ceres_stats["time_grad_eval"] = last_opt_summary_.gradient_evaluation_time_in_seconds;
  ceres_stats["n_iter"] = last_opt_summary_.iterations.size();
  ceres_stats["total_time"] = last_opt_summary_.total_time_in_seconds;
  ceres_stats["custom_solve_time"] = last_plan_time_ceres_solve_;
  ceres_stats["custom_logger_time"] = last_plan_time_logger_;
  std::ofstream ceres_stats_f(save_traj_abs_dir_ + "/ceres_summary.yaml");
  ceres_stats_f << ceres_stats;
  ceres_stats_f.close();

  VLOG(1) << "Done!";
}

template <typename T>
bool QuadTrajOpt<T>::setPlannerStateCallback(PlanConfig::Request& req,
                                             PlanConfig::Response& /*res*/)
{
  CHECK(rpg::fs::fileExists(req.config));
  resetPlannerState();
  loadFromYaml(req.config);
  if (calculate_info_from_pc_)
  {
    this->act_map_server_->getActMapRef().prepareInfoFromPointCloud();
  }
  setUpPolyTraj();

  return true;
}

template <typename T>
bool QuadTrajOpt<T>::playPlannedTrajCallback(std_srvs::Empty::Request& /*req*/,
                                             std_srvs::Empty::Response& /*res*/)
{
  if (!this->has_valid_traj_)
  {
    LOG(WARNING) << "Play traj: no valid traj, doing nothing";
    return false;
  }
  const act_map::ActMap<T>& map = this->act_map_server_->getActMapCRef();
  map.cachePointsAndViewDirs(false);
  rpg::PoseVec Twb_vec;
  std::vector<double> sampled_times;
  this->sampleTrajectory(options_.save_sample_dt_, &Twb_vec, &sampled_times);
  constexpr double viz_scale = 0.8;
  ros::Time now = ros::Time::now();
  Eigen::Vector3d rgb;
  rgb.setZero();
  rgb(1) = 1.0;

  PointCloud traj_pos_pc;
  pcl_conversions::toPCL(now, traj_pos_pc.header.stamp);
  traj_pos_pc.header.frame_id = PlannerBase<T>::kWorldFrame;
  traj_pos_pc.reserve(Twb_vec.size());
  for (const rpg::Pose& Twb : Twb_vec)
  {
    Point pt;
    pt.x = static_cast<float>(Twb.getPosition().x());
    pt.y = static_cast<float>(Twb.getPosition().y());
    pt.z = static_cast<float>(Twb.getPosition().z());
    pt.r = static_cast<uint8_t>(rgb(0) * 255);
    pt.g = static_cast<uint8_t>(rgb(1) * 255);
    pt.b = static_cast<uint8_t>(rgb(2) * 255);
    traj_pos_pc.push_back(pt);
  }
  play_traj_pos_pub_.publish(traj_pos_pc);

  ros::Duration(1.0).sleep();

  for (size_t i = 0; i < Twb_vec.size(); i++)
  {
    // coordinate frame
    ros::Duration(options_.save_sample_dt_).sleep();
    rpg::Pose Twc = Twb_vec[i] * Tbc_;
    visualization_msgs::MarkerArray ma_cam;
    act_map_ros::rotTransToAxisMarkerArray({ Twc }, "cam", viz_scale, &ma_cam);
    play_traj_orient_pub_.publish(ma_cam);

    // camera pyramid
    act_map_ros::publishCameraMarker(play_traj_orient_pub_, PlannerBase<T>::kWorldFrame, "cam_view",
                                     now, 0, visualization_msgs::Marker::ADD, viz_scale * 2.50, rgb,
                                     Twc);

    // viz link
    visualization_msgs::Marker viz_link;
    viz_link.id = 0;
    viz_link.ns = std::string("visible_lms");

    viz_link.header.frame_id = PlannerBase<T>::kWorldFrame;
    viz_link.header.stamp = now;
    viz_link.type = visualization_msgs::Marker::LINE_LIST;
    viz_link.action = visualization_msgs::Marker::ADD;

    viz_link.scale.x = options_.viz_link_size_;
    viz_link.scale.y = options_.viz_link_size_;
    viz_link.scale.z = options_.viz_link_size_;
    viz_link.color.a = 0.5;
    viz_link.color.r = rgb(0);
    viz_link.color.g = rgb(1);
    viz_link.color.b = rgb(2);
    act_map::VisIdx vis_idx;
    map.visCheckerCRef().getVisibleIdx(Twc, map.cachedPoints(), map.cachedViewDirs(), &vis_idx);
    for (const size_t& pt_idx : vis_idx)
    {
      const Eigen::Vector3d& pw_i = map.cachedPoints().at(pt_idx);
      geometry_msgs::Point point;
      point.x = static_cast<double>(Twc.getPosition()(0));
      point.y = static_cast<double>(Twc.getPosition()(1));
      point.z = static_cast<double>(Twc.getPosition()(2));
      viz_link.points.push_back(point);
      point.x = static_cast<double>(pw_i(0));
      point.y = static_cast<double>(pw_i(1));
      point.z = static_cast<double>(pw_i(2));
      viz_link.points.push_back(point);
    }
    play_traj_viz_link_pub_.publish(viz_link);
  }

  return true;
}

template <typename T>
bool PlanCeresFunction<T>::Evaluate(const double* params, double* cost, double* gradient) const
{
  VLOG(10) << "Enter Evaluate.";
  PolyParams pos_constraints, yaw_constraints;
  std::tie(pos_constraints, yaw_constraints) = traj_opt_->getVectorsOfPolyDerivSize();

  VLOG(10) << "Unpack params";
  traj_opt_->unpackCeresParams(params, true, &pos_constraints, &yaw_constraints);

  VLOG(10) << "Set constraints";
  traj_opt_->setFreeConstraints(pos_constraints, yaw_constraints);

  PolyParams pos_grad, yaw_grad;
  std::tie(pos_grad, yaw_grad) = traj_opt_->getVectorsOfPolyDerivSize();
  PolyParams* pos_grad_ptr = traj_opt_->ceresHasPosition() ? &pos_grad : nullptr;
  PolyParams* yaw_grad_ptr = traj_opt_->ceresHasYaw() ? &yaw_grad : nullptr;

  VLOG(10) << "Calculate cost and gradient.";
  (*cost) = traj_opt_->calculateCostAndGrad(pos_grad_ptr, yaw_grad_ptr);

  size_t offset = 0;
  if (traj_opt_->ceresHasPosition())
  {
    packVecVecXd(pos_grad, offset, gradient);
    offset += traj_opt_->pth_.n_free_ * 3;
  }
  if (traj_opt_->ceresHasYaw())
  {
    packVecVecXd(yaw_grad, offset, gradient);
    offset += traj_opt_->yth_.n_free_;
  }

  VLOG(10) << "Before exit";
  return true;
}

template <typename T>
int PlanCeresFunction<T>::NumParameters() const
{
  int num_params = 0;
  if (traj_opt_->ceresHasPosition())
  {
    num_params += traj_opt_->pos_poly_opt_.getNumberFreeConstraints() * 3;
  }
  if (traj_opt_->ceresHasYaw())
  {
    num_params += traj_opt_->yaw_poly_opt_.getNumberFreeConstraints();
  }
  return num_params;
}

template <typename T>
void LogTrajOpt<T>::init()
{
  traj_opt_->logger_weighted_cost_history_.clear();
  for (const auto& kv : traj_opt_->ceres_cost_to_weight_map_)
  {
    VLOG(1) << "LogTraj: add cost type " << kTrajCeresCostTToStr.at(kv.first) << " with weight "
            << kv.second;
    raw_cost_history_.insert({ kv.first, std::vector<double>() });
    w_cost_history_.insert({ kv.first, std::vector<double>() });
    traj_opt_->logger_weighted_cost_history_.insert({ kv.first, TimeValues() });
  }
  traj_opt_->logger_total_cost_history_.clear();
}

template <typename T>
ceres::CallbackReturnType LogTrajOpt<T>::operator()(const ceres::IterationSummary& /*summary*/)
{
  rpg::Timer timer_logger;
  timer_logger.start();
  VLOG(10) << "exe. iteration callback " << iter_cnt_;
  PolyParams pos_constraints, yaw_constraints;
  std::tie(pos_constraints, yaw_constraints) = traj_opt_->getVectorsOfPolyDerivSize();
  traj_opt_->unpackCeresParams(ceres_params_.data(), true, &pos_constraints, &yaw_constraints);
  traj_opt_->setFreeConstraints(pos_constraints, yaw_constraints);
  traj_opt_->initCeresCostCacheLastIter();
  traj_opt_->calculateCostAndGrad(nullptr, nullptr);

  VLOG(10) << "- save costs in iter. callback";
  double cur_total_cost = 0.0;
  for (const auto& kv : traj_opt_->ceres_last_raw_costs_)
  {
    CHECK(traj_opt_->ceres_cost_to_weight_map_.find(kv.first) !=
          traj_opt_->ceres_cost_to_weight_map_.end());
    raw_cost_history_[kv.first].push_back(kv.second);
    double w_cost = kv.second * traj_opt_->ceres_cost_to_weight_map_[kv.first];
    w_cost_history_[kv.first].push_back(w_cost);
    cur_total_cost += w_cost;
    traj_opt_->logger_weighted_cost_history_[kv.first].times.push_back(iter_cnt_);
    traj_opt_->logger_weighted_cost_history_[kv.first].values.push_back(w_cost);
  }
  traj_opt_->logger_total_cost_history_.times.push_back(iter_cnt_);
  traj_opt_->logger_total_cost_history_.values.push_back(cur_total_cost);

  if (traj_opt_->options_.viz_every_iter_pause_sec_ > 0)
  {
    VLOG(10) << "- visualize in iter. callback";
    traj_opt_->has_valid_traj_ = true;
    traj_opt_->visualize();
    if (traj_opt_->options_.visualize_iter_cnt_)
    {
      traj_opt_->publishText("Iterations: " + std::to_string(iter_cnt_),
                             Eigen::Vector3d(1.0, 1.0, 1.0), traj_opt_->text_pos_, "iter_cnt");
    }
    traj_opt_->has_valid_traj_ = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(traj_opt_->options_.viz_every_iter_pause_sec_ * 1000)));
  }

  iter_cnt_++;

  traj_opt_->last_plan_time_logger_ += timer_logger.stop();
  return ceres::SOLVER_CONTINUE;
}

template <typename T>
std::string LogTrajOpt<T>::summaryAsStr() const
{
  CHECK_EQ(raw_cost_history_.size(), w_cost_history_.size());
  std::vector<TrajCeresCostType> cost_types;
  std::vector<std::vector<double>> raw_costs;
  std::vector<std::vector<double>> w_costs;
  for (const auto& kv : raw_cost_history_)
  {
    cost_types.push_back(kv.first);
    raw_costs.push_back(kv.second);
    w_costs.push_back(w_cost_history_.at(kv.first));
  }
  const size_t n_iter = raw_costs.front().size();
  const std::string sep("\t\t");

  std::stringstream ss;
  ss << std::scientific;
  ss << "=== Cost each iteration ===\n";
  ss << "Iter." << sep;
  for (const TrajCeresCostType& t : cost_types)
  {
    ss << kTrajCeresCostTToStr.at(t) << sep;
  }
  ss << "||\t";
  for (const TrajCeresCostType& t : cost_types)
  {
    ss << kTrajCeresCostTToStr.at(t) << "-" << traj_opt_->ceres_cost_to_weight_map_.at(t) << sep;
  }
  ss << "tot. cost" << sep << "\n";
  for (size_t i = 0; i < n_iter; i++)
  {
    ss << i << sep;
    for (size_t ci = 0; ci < cost_types.size(); ci++)
    {
      ss << raw_costs[ci][i] << sep;
    }
    ss << "||\t";
    double tot = 0;
    for (size_t ci = 0; ci < cost_types.size(); ci++)
    {
      ss << w_costs[ci][i] << sep;
      tot += w_costs[ci][i];
    }
    CHECK_NEAR(tot, traj_opt_->logger_total_cost_history_.values[i], 1e-3);
    ss << "+-> " << tot << sep << "\n";
  }
  ss << "====================\n";
  return ss.str();
}

}  // namespace act_map_exp
