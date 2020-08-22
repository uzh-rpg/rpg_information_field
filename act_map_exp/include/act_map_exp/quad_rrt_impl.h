#pragma once

#include "act_map_exp/quad_rrt.h"
#include <yaml-cpp/yaml.h>
#include <rpg_common/fs.h>
#include <rpg_common_ros/params_helper.h>

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include "act_map_exp/ompl_utils.h"
#include "act_map_exp/exp_utils.h"
#include "act_map_exp/viz_utils.h"

namespace act_map_exp
{
template <typename T>
QuadRRT<T>::QuadRRT(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : PlannerBase<T>(nh, pnh)
{
  robot_radius_ = rpg_ros::param<double>(this->pnh_, "robot_radius", 0.5);
  XmlRpc::XmlRpcValue v;
  CHECK(this->pnh_.getParam("rrt_range", v));
  CHECK_EQ(6, v.size());
  VLOG(1) << "RRT ranges:";
  min_x_ = v[0];
  max_x_ = v[1];
  CHECK_GT(max_x_, min_x_);
  VLOG(1) << "- x: " << min_x_ << " ->" << max_x_;
  min_y_ = v[2];
  max_y_ = v[3];
  CHECK_GT(max_y_, min_y_);
  VLOG(1) << "- y: " << min_y_ << " -> " << max_y_;
  min_z_ = v[4];
  max_z_ = v[5];
  CHECK_GT(max_z_, min_z_);
  VLOG(1) << "- z: " << min_z_ << " -> " << max_z_;

  max_range_ = max_x_ - min_x_;
  max_range_ = max_range_ < max_y_ - min_y_ ? max_y_ - min_y_ : max_range_;
  max_range_ = max_range_ < max_z_ - min_z_ ? max_z_ - min_z_ : max_range_;
  VLOG(1) << "- max range: " << max_range_;

  clear_rrt_planner_srv_ = this->pnh_.advertiseService(
      "clear_rrt_planner", &QuadRRT::clearRRTPlannerCallback, this);
  red_.r = 1.0;
  red_.g = 0.0;
  red_.b = 0.0;
  red_.a = 1.0;
  green_.r = 0.0;
  green_.g = 1.0;
  green_.b = 0.0;
  green_.a = 1.0;
  gray_.r = 0.5;
  gray_.g = 0.5;
  gray_.b = 0.5;
  gray_.a = 1.0;
  rrt_vert_pub_ = this->pnh_.template advertise<PointCloud>("rrt_vert", 50);
  rrt_edge_pub_ =
      this->pnh_.template advertise<visualization_msgs::Marker>("rrt_edge", 50);

  std::cout << "===========================\n";
  std::cout << "=== Initialization Done ===\n";
  std::cout << "===========================\n";
}

template <typename T>
void QuadRRT<T>::plan()
{
  if (!planner_)
  {
    LOG(WARNING) << "Planner not initialized, forgot to set first?";
    return;
  }

  final_Twb_vec_.clear();
  consec_lower_than_thresh_cnt_ = 0;
  VLOG(1) << "Start RRT plan.";
  const size_t n_iter_prev = rrt_stats_.size();
  for (int cur_iter = 0; cur_iter < max_n_iter_; cur_iter++)
  {
    const int total_iter = cur_iter + 1 + n_iter_prev;
    VLOG(1) << "========= Iter. " << total_iter << " =========";
    ob::PlannerStatus solve_res = planner_->ob::Planner::solve(iter_time_sec_);
    if (solve_res)
    {
      CHECK(pdef_->hasSolution());
      has_valid_solution_ = true;

      if (!pdef_->hasExactSolution())
      {
        LOG(WARNING) << "Do not have exact solution.";
      }
      else
      {
        has_exact_solution_ = true;
      }

      const int cur_n_iter =
          planner_->as<ompl::geometric::RRTstar>()->numIterations();
      const double cur_best_cost =
          planner_->as<ompl::geometric::RRTstar>()->bestCost().value();
      planner_->as<ompl::geometric::RRTstar>()->getPlannerData(*planner_data_);

      VLOG(1) << "=====> Iter. " << total_iter << " succeeds: ";
      VLOG(1) << "- The best cost is " << cur_best_cost;
      VLOG(1) << "- The number of iteration is " << cur_n_iter;
      VLOG(1) << "- Has " << pdef_->getSolutionCount() << " solutions.";
      VLOG(1) << "- Has " << planner_data_->numEdges() << " edges.";
      VLOG(1) << "- Has " << planner_data_->numVertices() << " vertices.";
      const double prev_best_cost = rrt_stats_.lastBestCost();

      VLOG(1) << "- Log RRT stats...";
      rrt_stats_.n_iters_.push_back(cur_n_iter);
      rrt_stats_.best_costs_.push_back(cur_best_cost);
      rrt_stats_.n_verts_.push_back(planner_data_->numVertices());
      rrt_stats_.n_edges_.push_back(planner_data_->numEdges());
      act_map::Vec3dVec vertices;
      ompl_utils::getVerticesFromPlannerData(planner_data_, &vertices);
      Eigen::MatrixX3d vert_mat;
      act_map::VecKVecToEigenXK(vertices, &vert_mat);
      rrt_stats_.vertices_.push_back(vert_mat);
      Eigen::MatrixX2i pairs;
      pairs.resize(planner_data_->numEdges(), Eigen::NoChange);
      int pair_i = 0;
      for (size_t start_i = 0; start_i < planner_data_->numVertices();
           start_i++)
      {
        std::vector<unsigned int> end_indices;
        planner_data_->getEdges(start_i, end_indices);
        for (size_t end_i = 0; end_i < end_indices.size(); end_i++)
        {
          pairs(pair_i, 0) = static_cast<int>(start_i);
          pairs(pair_i, 1) = static_cast<int>(end_i);
          pair_i++;
        }
      }
      rrt_stats_.edge_pairs_.push_back(pairs);

      if (!std::isfinite(prev_best_cost) || !std::isfinite(cur_best_cost))
      {
        VLOG(1) << "- Either of the prev. or cur. cost is not finite.";
        consec_lower_than_thresh_cnt_ = 0;
      }
      else
      {
        const double ratio =
            std::fabs(cur_best_cost - prev_best_cost) / prev_best_cost;
        VLOG(1) << "- Cost reduction is " << ratio;
        if (ratio < converge_ratio_)
        {
          consec_lower_than_thresh_cnt_++;
        }
        else
        {
          consec_lower_than_thresh_cnt_ = 0;
        }
      }

      constexpr int kMinConsecRed = 2;
      VLOG(1) << "- consecutive reduction cnt. "
              << consec_lower_than_thresh_cnt_;
      if (cur_iter >= min_n_iter_)
      {
        if (consec_lower_than_thresh_cnt_ >= kMinConsecRed)
        {
          VLOG(1) << "Consecutive " << kMinConsecRed
                  << " iterations reduce the cost under threshold, stop.";
          break;
        }
      }

      if (viz_every_outer_iter_)
      {
        VLOG(1) << "Visualize current results...";
        extractRRTPathPoses();
        visualize();
      }
    }
    else
    {
      LOG(WARNING) << "XXXXX> Solve failed for iter " << total_iter;
    }
  }

  if (has_valid_solution_)
  {
    VLOG(1) << "Solve successfully!";
    extractRRTPathPoses();
    VLOG(1) << "Get " << final_Twb_vec_.size() << " vertices.";
  }
}

template <typename T>
void QuadRRT<T>::extractRRTPathPoses() const
{
  if (!has_valid_solution_)
  {
    return;
  }

  final_Twb_vec_.clear();
  ob::PathPtr path = pdef_->getSolutionPath();
  const std::vector<ob::State*>& states =
      path->as<ompl::geometric::PathGeometric>()->getStates();
  final_Twb_vec_.reserve(states.size());
  for (const ob::State* s : states)
  {
    Eigen::Vector3d pos;
    double yaw;
    ompl_utils::flatStateToPositionYaw(*s, &pos, &yaw);
    Eigen::Matrix3d rot_mat;
    quadAccYawToRwb(Eigen::Vector3d::Zero(), yaw, &rot_mat, nullptr, nullptr);
    final_Twb_vec_.emplace_back(rpg::Pose(rpg::Rotation(rot_mat), pos));
  }
}

template <typename T>
void QuadRRT<T>::visualize()
{
  if (!has_valid_solution_)
  {
    LOG(WARNING) << "No valid solution, skip visualizaiton.";
    return;
  }
  VLOG(1) << "start visualizaiton...";
  clearMarkerArray(this->traj_orient_pub_);
  visualizeTrajectory(
      final_Twb_vec_, this->traj_pos_pub_, this->traj_orient_pub_, Tbc_,
      Eigen::Vector3d(0.0, 1.0, 0.0), PlannerBase<T>::kWorldFrame);
  visualizeRRTPath();
  visualizeRRTVertsEdges();
}

template <typename T>
void QuadRRT<T>::saveResults()
{
  if (!has_valid_solution_)
  {
    LOG(WARNING) << "No valid solution, skip saving.";
    return;
  }

  if (save_abs_dir_.empty())
  {
    LOG(WARNING) << "Saving dir empty, skip.";
    return;
  }
  VLOG(1) << "start saving...";

  VLOG(1) << "- poses";
  const size_t n_pose = final_Twb_vec_.size();
  std::vector<double> dummy_time(n_pose);
  rpg::PoseVec Twc_vec(n_pose);
  unrealcv_bridge::UEPoseVec Twc_vec_ue(n_pose);
  for (size_t idx = 0; idx < final_Twb_vec_.size(); idx++)
  {
    Twc_vec[idx] = final_Twb_vec_[idx] * Tbc_;
    unrealcv_bridge::TwcToUEPose(Twc_vec[idx], &(Twc_vec_ue[idx]));
    dummy_time[idx] = static_cast<double>(idx);
  }
  saveStampedPoses(dummy_time, final_Twb_vec_,
                   save_abs_dir_ + "/" + PlannerBase<T>::kSaveTwbNm);
  saveStampedPoses(dummy_time, Twc_vec,
                   save_abs_dir_ + "/" + PlannerBase<T>::kSaveTwcNm);
  saveStampedPoses(dummy_time, Twc_vec_ue,
                   save_abs_dir_ + "/" + PlannerBase<T>::kSaveTwcUENm);

  VLOG(1) << "- number of features";
  std::vector<size_t> n_ftrs(n_pose);
  const act_map::ActMap<T>& map = this->act_map_server_->getActMapCRef();
  map.cachePointsAndViewDirs(true);
  for (size_t Tidx = 0; Tidx < Twc_vec.size(); Tidx++)
  {
    const rpg::Pose& Twc = Twc_vec[Tidx];
    act_map::VisIdx vis_idx;
    map.visCheckerCRef().getVisibleIdx(Twc, map.cachedPoints(),
                                       map.cachedViewDirs(), &vis_idx);
    n_ftrs[Tidx] = vis_idx.size();
  }
  saveStampedScalars(dummy_time, n_ftrs, save_abs_dir_ + "/stamped_n_ftrs.txt");

  VLOG(1) << "- RRT related";
  rrt_stats_.save(save_abs_dir_);
}

template <typename T>
void QuadRRT<T>::resetPlannerState()
{
  start_pos_.setZero();
  end_pos_.setZero();
  start_yaw_rad_ = 0.0;
  end_yaw_rad_ = 0.0;

  iter_time_sec_ = -1.0;
  max_n_iter_ = -1;
  min_n_iter_ = -1;
  converge_ratio_ = -1.0;

  flat_ss_.reset();
  flat_si_.reset();
  pdef_.reset();
  planner_.reset();
  planner_data_.reset();

  use_esdf_checker_ = false;
  use_joint_checker_ = false;

  Tbc_.setIdentity();
  rrt_cost_to_weight_map_.clear();
  rrt_range_ = -1.0;

  final_Twb_vec_.clear();
  rrt_stats_.clear();
  consec_lower_than_thresh_cnt_ = 0;
  has_valid_solution_ = false;
  has_exact_solution_ = false;
}

template <typename T>
void QuadRRT<T>::clearRRT()
{
  CHECK(planner_);
  VLOG(1) << "reset RRT state (the planner config. does not change)";
  rrt_stats_.clear();
  consec_lower_than_thresh_cnt_ = 0;
  planner_->as<ompl::geometric::RRTstar>()->clear();
  planner_data_->clear();
  has_valid_solution_ = false;
  has_exact_solution_ = false;
}

template <typename T>
bool QuadRRT<T>::clearRRTPlannerCallback(std_srvs::Empty::Request& /*req*/,
                                         std_srvs::Empty::Response& /*res*/)
{
  clearRRT();
  return true;
}

template <typename T>
bool QuadRRT<T>::setPlannerStateCallback(PlanConfig::Request& req,
                                         PlanConfig::Response& /*res*/)
{
  CHECK(rpg::fs::fileExists(req.config));
  resetPlannerState();
  loadFromYaml(req.config);
  if (cal_info_from_pc_)
  {
    this->act_map_server_->getActMapRef().prepareInfoFromPointCloud();
  }
  setupRRT();

  return true;
}

template <typename T>
void QuadRRT<T>::loadFromYaml(const std::string& param_fn)
{
  VLOG(1) << "Loading parameters from " << param_fn;
  YAML::Node node = YAML::LoadFile(param_fn);
  CHECK(node.IsMap());

  CHECK(node["start"]);
  CHECK(node["start"].IsSequence());
  start_pos_.x() = node["start"][0].as<double>();
  start_pos_.y() = node["start"][1].as<double>();
  start_pos_.z() = node["start"][2].as<double>();

  CHECK(node["end"]);
  CHECK(node["end"].IsSequence());
  end_pos_.x() = node["end"][0].as<double>();
  end_pos_.y() = node["end"][1].as<double>();
  end_pos_.z() = node["end"][2].as<double>();

  CHECK(inRange(start_pos_));
  CHECK(inRange(end_pos_));
  VLOG(1) << "Planning from " << start_pos_.transpose() << " to "
          << end_pos_.transpose();

  constexpr double kDegToRad = M_PI / 180.0;
  CHECK(node["start_yaw_deg"]);
  start_yaw_rad_ = node["start_yaw_deg"].as<double>() * kDegToRad;
  CHECK(node["end_yaw_deg"]);
  end_yaw_rad_ = node["end_yaw_deg"].as<double>() * kDegToRad;
  VLOG(1) << "Yaw is from " << start_yaw_rad_ << " to " << end_yaw_rad_;

  CHECK(node["use_esdf_checker"]);
  use_esdf_checker_ = node["use_esdf_checker"].as<bool>();
  VLOG(1) << "Use ESDF for state validity: " << asStr(use_esdf_checker_);

  CHECK(node["use_joint_checker"]);
  use_joint_checker_ = node["use_joint_checker"].as<bool>();
  VLOG(1) << "Use joint checker: " << asStr(use_joint_checker_);

  CHECK(node["iter_time_sec"]);
  iter_time_sec_ = node["iter_time_sec"].as<double>();
  CHECK_GT(iter_time_sec_, 0.0);
  VLOG(1) << "Allow planner to take " << iter_time_sec_
          << " seconds each iteration.";

  CHECK(node["max_n_iter"]);
  max_n_iter_ = node["max_n_iter"].as<int>();
  CHECK_GT(max_n_iter_, 0);
  VLOG(1) << "Allow planner to run max. " << max_n_iter_
          << " iterations outer loop";

  CHECK(node["min_n_iter"]);
  min_n_iter_ = node["min_n_iter"].as<int>();
  CHECK_GE(min_n_iter_, 0);
  VLOG(1) << "Allow planner to run min. " << min_n_iter_
          << " iterations outer loop";

  CHECK(node["converge_ratio"]);
  converge_ratio_ = node["converge_ratio"].as<double>();
  CHECK_GT(converge_ratio_, 0);
  VLOG(1) << "Converge ratio (of cost reduction) is " << converge_ratio_;

  CHECK(node["viz_every_outer_iter"]);
  viz_every_outer_iter_ = node["viz_every_outer_iter"].as<bool>();
  VLOG(1) << "Will visualize every iteration: " << asStr(viz_every_outer_iter_);

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

  CHECK(node["rrt_cost"]);
  for (size_t ci = 0; ci < node["rrt_cost"].size(); ci++)
  {
    for (const auto& cost_pair : node["rrt_cost"][ci])
    {
      const std::string cost_nm = cost_pair.first.as<std::string>();
      CHECK(kStrToRRTCostT.find(cost_nm) != kStrToRRTCostT.end());
      const double weight = cost_pair.second.as<double>();
      VLOG(1) << "Adding cost " << cost_nm << " with weight " << weight;
      rrt_cost_to_weight_map_.insert({ kStrToRRTCostT.at(cost_nm), weight });
    }
  }

  if (node["rrt_range"])
  {
    rrt_range_ = node["rrt_range"].as<double>();
    VLOG(1) << "RRT range is set to " << rrt_range_;
  }
  else
  {
    LOG(WARNING) << "Using default rrt range.";
  }

  CHECK(node["calculate_info_from_pc"]);
  cal_info_from_pc_ = node["calculate_info_from_pc"].as<bool>();
  VLOG(1) << "Will calcualte information from point cloud: "
          << asStr(cal_info_from_pc_);
  CHECK(node["info_metric_type"]);
  const std::string info_type_str = node["info_metric_type"].as<std::string>();
  CHECK(act_map::kNameToInfoMetric.find(info_type_str) !=
        act_map::kNameToInfoMetric.end());
  VLOG(1) << "Information type: " << info_type_str;
  info_metric_type_ = act_map::kNameToInfoMetric.at(info_type_str);

  CHECK(node["save_abs_dir"]);
  save_abs_dir_ = node["save_abs_dir"].as<std::string>();
  CHECK(rpg::fs::pathExists((save_abs_dir_)))
      << save_abs_dir_ << " does not exist.";
  VLOG(1) << "Saving results to " << save_abs_dir_;

  bool reinit_info_pot = false;
  act_map::InfoPotentialOptions opts = this->info_pot_ptr_->getOptionsCopy();
  if (node["info_pot_n_random_lm"])
  {
    const int rand_lm = node["info_pot_n_random_lm"].as<int>();
    if (rand_lm != opts.n_random_landmarks_)
    {
      opts.n_random_landmarks_ = rand_lm;
      VLOG(1) << "Found new number of landmarks: " << opts.n_random_landmarks_;
      reinit_info_pot = true;
    }
  }
  if (node["info_pot_min_depth_m"])
  {
    const double min_d = node["info_pot_min_depth_m"].as<double>();
    if (std::fabs(min_d - opts.min_depth_m_) > 1e-3)
    {
      opts.min_depth_m_ = min_d;
      VLOG(1) << "Found new min-depth: " << opts.min_depth_m_;
      reinit_info_pot = true;
    }
  }
  if (node["info_pot_max_depth_m"])
  {
    const double max_d = node["info_pot_max_depth_m"].as<double>();
    if (std::fabs(max_d - opts.max_depth_m_) > 1e-3)
    {
      opts.max_depth_m_ = max_d;
      VLOG(1) << "Found new max depth: " << opts.max_depth_m_;
      reinit_info_pot = true;
    }
  }
  if (reinit_info_pot)
  {
    VLOG(1) << "Reinit information potential...";
    this->info_pot_ptr_->reinit(opts);
    VLOG(1) << *(this->info_pot_ptr_);
  }
  else
  {
    VLOG(1) << "Info. potential settings do not change, skip.";
  }
}

template <typename T>
void QuadRRT<T>::visualizeRRTVertsEdges() const
{
  if (!planner_data_)
  {
    LOG(WARNING) << "No planner data available.";
    return;
  }

  act_map::Vec3dVec vert_pos;
  ompl_utils::getVerticesFromPlannerData(planner_data_, &vert_pos);

  ros::Time now = ros::Time::now();
  rpg::Timer timer;
  if (rrt_vert_pub_.getNumSubscribers() > 0)
  {
    timer.start();
    PointCloud rrt_pc;
    pcl_conversions::toPCL(now, rrt_pc.header.stamp);
    rrt_pc.header.frame_id = PlannerBase<T>::kWorldFrame;
    rrt_pc.reserve(vert_pos.size());
    for (const Eigen::Vector3d& vert : vert_pos)
    {
      Point pt;
      pt.x = static_cast<float>(vert.x());
      pt.y = static_cast<float>(vert.y());
      pt.z = static_cast<float>(vert.z());
      pt.r = 0;
      pt.g = 0;
      pt.b = 255;
      rrt_pc.push_back(pt);
    }
    VLOG(1) << "Publish " << rrt_pc.size() << " RRT vertices: " << timer.stop()
            << " sec.";
    rrt_vert_pub_.publish(rrt_pc);
  }

  if (rrt_edge_pub_.getNumSubscribers() > 0)
  {
    constexpr double kMinEdgelen = 0.01;
    clearMarker(rrt_edge_pub_);
    timer.start();
    visualization_msgs::Marker edges;
    edges.id = 0;
    edges.ns = "rrt_edge";

    edges.header.frame_id = PlannerBase<T>::kWorldFrame;
    edges.header.stamp = ros::Time::now();
    edges.type = visualization_msgs::Marker::LINE_LIST;
    edges.action = visualization_msgs::Marker::ADD;

    edges.scale.x = 0.05;
    edges.color.a = 0.3;
    edges.color.r = 1.0;
    edges.color.g = 1.0;
    edges.color.b = 0.0;
    edges.points.reserve(planner_data_->numEdges());
    for (size_t vert_i = 0; vert_i < planner_data_->numVertices(); vert_i++)
    {
      geometry_msgs::Point s;
      s.x = vert_pos[vert_i].x();
      s.y = vert_pos[vert_i].y();
      s.z = vert_pos[vert_i].z();
      std::vector<unsigned int> end_indices;
      planner_data_->getEdges(vert_i, end_indices);
      for (unsigned int end_i : end_indices)
      {
        if ((vert_pos[end_i] - vert_pos[vert_i]).norm() < kMinEdgelen)
        {
          continue;
        }
        geometry_msgs::Point e;
        e.x = vert_pos[end_i].x();
        e.y = vert_pos[end_i].y();
        e.z = vert_pos[end_i].z();
        edges.points.push_back(s);
        edges.points.push_back(e);
      }
    }
    VLOG(1) << "Pubish " << edges.points.size()
            << " RRT edges: " << timer.stop() << " sec.";
    rrt_edge_pub_.publish(edges);
  }
}

template <typename T>
void QuadRRT<T>::visualizeRRTPath() const
{
  if (has_exact_solution_)
  {
    visualizePath(final_Twb_vec_, red_, green_, this->general_marker_pub_, 0,
                  PlannerBase<T>::kWorldFrame, "rrt_path");
  }
  else
  {
    visualizePath(final_Twb_vec_, gray_, gray_, this->general_marker_pub_, 0,
                  PlannerBase<T>::kWorldFrame, "rrt_path");
  }
}

template <typename T>
void QuadRRT<T>::setupRRT()
{
  VLOG(1) << "Setting up RRT...";
  VLOG(1) << "- state space";
  ob::StateSpacePtr pos_ss(new ob::RealVectorStateSpace(3));
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, min_x_);
  bounds.setHigh(0, max_x_);
  bounds.setLow(1, min_y_);
  bounds.setHigh(1, max_y_);
  bounds.setLow(2, min_z_);
  bounds.setHigh(2, max_z_);
  pos_ss->as<ob::RealVectorStateSpace>()->setBounds(bounds);
  ob::StateSpacePtr yaw_ss(new ob::SO2StateSpace());
  flat_ss_.reset(new ob::CompoundStateSpace());
  flat_ss_->as<ob::CompoundStateSpace>()->addSubspace(pos_ss, 1.0);
  flat_ss_->as<ob::CompoundStateSpace>()->addSubspace(yaw_ss,
                                                      kYawDiscountFactor);

  VLOG(1) << "- state space information";
  flat_si_.reset(new ob::SpaceInformation(flat_ss_));

  bool info_used_for_validity_checking = false;
  if (use_joint_checker_ &&
      (this->hasValidESDFMap() && this->hasValidInfoMap()))
  {
    VLOG(1) << " using ESDF + Info for validity check.";
    flat_si_->setStateValidityChecker(
        ob::StateValidityCheckerPtr(new ompl_utils::JointChecker<T>(
            flat_si_,
            std::const_pointer_cast<const voxblox::EsdfServer>(
                this->esdf_server_)
                ->getEsdfMapPtr(),
            robot_radius_, this->act_map_server_->getActMapCRef(),
            this->info_pot_ptr_, Tbc_, info_metric_type_, cal_info_from_pc_)));
    info_used_for_validity_checking = true;
  }
  else if (use_esdf_checker_ && this->hasValidESDFMap())
  {
    VLOG(1) << " using ESDF for validity check.";
    flat_si_->setStateValidityChecker(
        ob::StateValidityCheckerPtr(new ompl_utils::ESDFChecker(
            flat_si_,
            std::const_pointer_cast<const voxblox::EsdfServer>(
                this->esdf_server_)
                ->getEsdfMapPtr(),
            robot_radius_)));
  }
  else
  {
    LOG(WARNING) << "No esdf map found, will use a dummy checker";
    flat_si_->setStateValidityChecker(
        ob::StateValidityCheckerPtr(new ompl_utils::DummyChecker(flat_si_)));
  }

  flat_si_->setStateValidityCheckingResolution(this->calValidityResolution());
  flat_si_->setup();

  VLOG(1) << "- problem definition";
  VLOG(1) << "  distance between goal and start positions: "
          << (start_pos_ - end_pos_).norm();
  pdef_.reset(new ob::ProblemDefinition(flat_si_));
  ob::ScopedState<> start(flat_ss_);
  ompl_utils::positionYawToFlatState(start_pos_, start_yaw_rad_, start.get());
  ob::ScopedState<> end(flat_ss_);
  ompl_utils::positionYawToFlatState(end_pos_, end_yaw_rad_, end.get());
  pdef_->setStartAndGoalStates(start, end, 1.0);

  VLOG(1) << "- planner";
  ob::OptimizationObjectivePtr total_cost(
      new ob::MultiOptimizationObjective(flat_si_));
  for (const auto& cost_pair : rrt_cost_to_weight_map_)
  {
    if (cost_pair.first == RRTCostType::kDist)
    {
      std::dynamic_pointer_cast<ob::MultiOptimizationObjective>(total_cost)
          ->addObjective(ob::OptimizationObjectivePtr(
                             new ompl_utils::DistanceCost(flat_si_)),
                         cost_pair.second);
    }
    else if (cost_pair.first == RRTCostType::kYaw)
    {
      std::dynamic_pointer_cast<ob::MultiOptimizationObjective>(total_cost)
          ->addObjective(
              ob::OptimizationObjectivePtr(new ompl_utils::YawCost(flat_si_)),
              cost_pair.second);
    }
    else if (cost_pair.first == RRTCostType::kInfo)
    {
      if (!this->hasValidInfoMap())
      {
        LOG(WARNING) << "No info. map exists, skip.";
        continue;
      }
      if (info_used_for_validity_checking)
      {
        LOG(WARNING) << "Already used info. for validity checking, skip.";
        continue;
      }
      CHECK(this->info_pot_ptr_);
      CHECK(info_metric_type_ != act_map::InfoMetricType::kNone);
      std::dynamic_pointer_cast<ob::MultiOptimizationObjective>(total_cost)
          ->addObjective(
              ob::OptimizationObjectivePtr(new ompl_utils::InfoCost<T>(
                  flat_si_, this->act_map_server_->getActMapCRef(),
                  this->info_pot_ptr_, info_metric_type_, Tbc_,
                  cal_info_from_pc_)),
              cost_pair.second);
    }
    else if (cost_pair.first == RRTCostType::kESDF)
    {
      LOG(FATAL) << "ESDF as optimiztaion cost not supported.";
    }
    else
    {
      LOG(FATAL) << "Unknown cost type ";
    }
  }
  pdef_->setOptimizationObjective(total_cost);
  total_cost->print(std::cout);

  planner_.reset(new ompl::geometric::RRTstar(flat_si_));
  std::dynamic_pointer_cast<ompl::geometric::RRTstar>(planner_)->setRange(
      rrt_range_);
  planner_->setProblemDefinition(pdef_);
  planner_->setup();

  planner_data_.reset(new ob::PlannerData(flat_si_));

  VLOG(1) << "=== Summary of problem and planner ===";
  VLOG(1) << "- problem def.:";
  pdef_->print();
  VLOG(1) << "- planner settings: ";
  planner_->printSettings(std::cout);
  VLOG(1) << "- planner prop.:";
  planner_->printProperties(std::cout);
}

template <typename T>
double QuadRRT<T>::calValidityResolution() const
{
  constexpr double def_vox_size = 0.1;
  if (this->hasValidESDFMap())
  {
    return this->esdf_server_->getEsdfMapPtr()->voxel_size() / max_range_;
  }
  else
  {
    return def_vox_size / max_range_;
  }
}

}  // namespace act_map_exp
