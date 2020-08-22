#pragma once

#include "act_map_exp/planner_base.h"

#include <tuple>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ceres/ceres.h>

#include "act_map_exp/PlanConfig.h"
#include "act_map_exp/potential_cost.h"
#include "act_map_exp/exp_utils.h"
#include "act_map_exp/poly_traj_helper.h"

namespace mtg = mav_trajectory_generation;

namespace act_map_exp
{
struct QuadTrajOptOptions
{
  double integral_cost_sample_dt_ = 0.5;
  double robot_radius_ = 0.5;
  double col_margin_ = 0.5;

  int waypoint_zero_deriv_max_ = 2;  // velocity
  double viz_every_iter_pause_sec_ = -1.0;
  int viz_observations_every_n_ = 1;
  double viz_link_size_ = 0.02;

  // info related
  act_map::InfoMetricType info_metric_type_ = act_map::InfoMetricType::kTrace;

  // ceres
  double ceres_parameter_tolerance_ = 1e-8;
  double ceres_function_tolerance_ = 1e-6;
  int ceres_max_iter_ = 50;
  bool second_stage_opt_yaw_only_ = false;

  ceres::LineSearchDirectionType ceres_line_search_type_;
  bool ceres_use_logger_ = false;

  // visualizaiton
  bool visualize_visible_points_ = false;
  bool visualize_iter_cnt_ = false;
  double text_scale_ = 10.0;

  // misc.
  bool info_metric_use_log_det_ = true;
  double save_sample_dt_ = 0.1;
};

extern const std::unordered_map<std::string, ceres::LineSearchDirectionType> kStrToCeresLSType;

enum class TrajOptType
{
  kUnknown,
  kLinear,
  kCeres
};

extern const std::unordered_map<std::string, TrajOptType> kStrToTrajOptT;

enum class TrajCeresCostType
{
  kPosDynamic,
  kYawDynamic,
  kESDF,
  kInfo
};

inline bool isIntegralType(const TrajCeresCostType& ct)
{
  return ct == TrajCeresCostType::kESDF || ct == TrajCeresCostType::kInfo;
}

extern const std::unordered_map<std::string, TrajCeresCostType> kStrToTrajCeresCostT;
extern const std::unordered_map<TrajCeresCostType, std::string> kTrajCeresCostTToStr;

enum class TrajCeresOptType
{
  kPosition,
  kYaw,
  kJoint,
  kTwoStages
};
extern const std::unordered_map<std::string, TrajCeresOptType> kStrToTrajCeresOptType;

using CeresCostValMap = std::unordered_map<TrajCeresCostType, double>;
using CeresCostTimeValMap = std::unordered_map<TrajCeresCostType, TimeValues>;

template <typename T>
class PlanCeresFunction;

template <typename T>
class LogTrajOpt;

template <typename T>
class QuadTrajOpt : public PlannerBase<T>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  constexpr static int kPosPolyOrder = 10;
  constexpr static int kYawPolyOrder = 6;
  constexpr static int kPosDerivOptim = mtg::derivative_order::SNAP;
  constexpr static int kYawDerivOptim = mtg::derivative_order::ACCELERATION;

  QuadTrajOpt() = delete;
  QuadTrajOpt(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  void plan() override;

  void visualize() override;

  void saveResults() override;

  void resetPlannerState() override;

  QuadTrajOptOptions options_;

private:
  bool setPlannerStateCallback(PlanConfig::Request& req, PlanConfig::Response& res) override;
  bool playPlannedTrajCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  enum class CeresParamType
  {
    kNone,
    kPosition,
    kYaw,
    kPositionYaw
  };
  void readROSParams();
  void loadFromYaml(const std::string& param_fn);

  // polynomial is used as the underlying representation, so we need to setup
  // this anyway no matter what kind of solver is used.
  void setUpPolyTraj();

  void planLinear();

  void planCeres();

  double ceresSolve(PolyParams* pos_free_constraints, PolyParams* yaw_free_constraints);

  void sampleAndVisualizeCurrentTraj(const ros::Publisher& traj_pos_pub,
                                     const ros::Publisher& traj_orient_pub,
                                     const Eigen::Vector3d& rgb,
                                     const std::string& marker_ns) const;

  void publishText(const std::string& text, const Eigen::Vector3d& rgb, const Eigen::Vector3d& pos,
                   const std::string& ns) const;

  inline std::tuple<PolyParams, PolyParams> getVectorsOfPolyDerivSize() const
  {
    return std::make_tuple(
        PolyParams(3, Eigen::VectorXd(pos_poly_opt_.getNumberFreeConstraints())),
        PolyParams(1, Eigen::VectorXd(yaw_poly_opt_.getNumberFreeConstraints())));
  }

  inline bool ceresHasPosition() const
  {
    return ceres_param_type_ == CeresParamType::kPosition ||
           ceres_param_type_ == CeresParamType::kPositionYaw;
  }

  inline bool ceresHasYaw() const
  {
    return ceres_param_type_ == CeresParamType::kYaw ||
           ceres_param_type_ == CeresParamType::kPositionYaw;
  }

  double calculateCostAndGrad(PolyParams* pos_grad, PolyParams* yaw_grad) const;

  void calculateIntegralCost(const std::vector<TrajCeresCostType>& types,
                             std::vector<double>* costs, std::vector<PolyParams>* pos_grad,
                             std::vector<PolyParams>* yaw_grad) const;

  void sampleTrajectory(const double sample_dt, mav_msgs::EigenTrajectoryPointVector* states,
                        std::vector<double>* yaws_rad) const;

  void sampleTrajectory(const double sample_dt, rpg::PoseVec* Twb_vec,
                        std::vector<double>* times_sec = nullptr) const;

  inline void unpackCeresParams(const double* params, const bool clear_unused,
                                PolyParams* pos_constraints, PolyParams* yaw_constraints) const
  {
    size_t offset = 0;
    if (this->ceresHasPosition())
    {
      unpackVecToVecVecXd(params, offset, pos_constraints);
      offset += 3 * this->pth_.n_free_;
    }
    else if (clear_unused)
    {
      pos_constraints->clear();
    }

    if (this->ceresHasYaw())
    {
      unpackVecToVecVecXd(params, offset, yaw_constraints);
      offset += this->yth_.n_free_;
    }
    else if (clear_unused)
    {
      yaw_constraints->clear();
    }
  }

  inline void setFreeConstraints(const PolyParams& pos_constraints,
                                 const PolyParams& yaw_constraints)
  {
    if (!pos_constraints.empty())
    {
      pos_poly_opt_.setFreeConstraints(pos_constraints);
    }
    if (!yaw_constraints.empty())
    {
      yaw_poly_opt_.setFreeConstraints(yaw_constraints);
    }
  }

  inline void getFreeConstraints(PolyParams* pos_constraints, PolyParams* yaw_constraints)
  {
    if (pos_constraints)
    {
      pos_poly_opt_.getFreeConstraints(pos_constraints);
    }
    if (yaw_constraints)
    {
      yaw_poly_opt_.getFreeConstraints(yaw_constraints);
    }
  }

  void initCeresCostCacheLastIter() const;

  ros::Publisher traj_orient_pub_stageone_;
  ros::Publisher traj_pos_pub_stageone_;
  ros::Publisher failed_sample_points_pub_;

  ros::ServiceServer play_traj_srv_;
  ros::Publisher play_traj_orient_pub_;
  ros::Publisher play_traj_pos_pub_;
  ros::Publisher play_traj_viz_link_pub_;

  mtg::PolynomialOptimization<kPosPolyOrder> pos_poly_opt_;
  mtg::PolynomialOptimization<kYawPolyOrder> yaw_poly_opt_;

  // planner state
  TrajOptType traj_opt_type_ = TrajOptType::kUnknown;
  TrajCeresOptType ceres_opt_type_ = TrajCeresOptType::kJoint;
  CeresParamType ceres_param_type_ = CeresParamType::kNone;
  act_map::Vec3dVec pos_wps_;
  std::vector<double> yaw_wps_;
  std::vector<int> num_segments_;
  double total_time_sec_ = 10.0;
  CeresCostValMap ceres_cost_to_weight_map_;
  rpg::Pose Tbc_;
  bool calculate_info_from_pc_ = false;
  std::string save_traj_abs_dir_ = std::string("");

  // caches every iteration for visualization and saving
  mutable CeresCostValMap ceres_last_raw_costs_;
  mutable CeresCostTimeValMap ceres_last_integral_cost_samples_;
  mutable act_map::Vec3dVec integral_points_;
  mutable act_map::Vec3dVec failed_sample_points_;

  // indicate whether there is a valid trajectory, regardless of the method
  bool has_valid_traj_ = false;

  // different potential costs
  PotentialCostPtr pot_;

  // information for last planning (ceres)
  ceres::GradientProblemSolver::Summary last_opt_summary_;
  double last_plan_time_ceres_solve_ = 0.0;
  double last_plan_time_logger_ = 0.0;
  // filled by the logger
  CeresCostTimeValMap logger_weighted_cost_history_;
  TimeValues logger_total_cost_history_;

  mutable PolyTrajHelper<kPosPolyOrder, 3> pth_;
  mutable PolyTrajHelper<kYawPolyOrder, 1> yth_;

  friend class PlanCeresFunction<T>;
  friend class LogTrajOpt<T>;

  // misc
  Eigen::Vector3d text_pos_;
};

template <typename T>
constexpr int QuadTrajOpt<T>::kPosPolyOrder;

template <typename T>
constexpr int QuadTrajOpt<T>::kYawPolyOrder;

template <typename T>
constexpr int QuadTrajOpt<T>::kPosDerivOptim;

template <typename T>
constexpr int QuadTrajOpt<T>::kYawDerivOptim;

template <typename T>
class PlanCeresFunction : public ceres::FirstOrderFunction
{
public:
  PlanCeresFunction(QuadTrajOpt<T>* quad_opt) : traj_opt_(quad_opt)
  {
  }

  virtual bool Evaluate(const double* parameters, double* cost, double* gradient) const;

  virtual int NumParameters() const;

private:
  QuadTrajOpt<T>* traj_opt_;
};

template <typename T>
class LogTrajOpt : public ceres::IterationCallback
{
public:
  LogTrajOpt(QuadTrajOpt<T>* quad_opt, const std::vector<double>& ceres_params)
    : traj_opt_(quad_opt), ceres_params_(ceres_params)
  {
    init();
  }

  virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);
  void init();
  std::string summaryAsStr() const;

private:
  QuadTrajOpt<T>* traj_opt_;
  const std::vector<double>& ceres_params_;
  size_t iter_cnt_ = 0;
  std::unordered_map<TrajCeresCostType, std::vector<double>> raw_cost_history_;
  std::unordered_map<TrajCeresCostType, std::vector<double>> w_cost_history_;
};

}  // namespace act_map_exp

#include "act_map_exp/quad_traj_opt_impl.h"
