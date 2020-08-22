#pragma once

#include "act_map_exp/planner_base.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>

namespace ob = ompl::base;

namespace act_map_exp
{

enum class RRTCostType
{
  kDist,
  kYaw,
  kInfo,
  kESDF  // ESDF will probably be used as validity checker
};

struct RRTStats
{
  std::vector<int> n_iters_;
  std::vector<double> best_costs_;
  std::vector<int> n_verts_;
  std::vector<int> n_edges_;

  std::vector<Eigen::MatrixX3d> vertices_;
  std::vector<Eigen::MatrixX2i> edge_pairs_;
  inline void clear()
  {
    n_iters_.clear();
    best_costs_.clear();
    n_verts_.clear();
    n_edges_.clear();
    vertices_.clear();
    edge_pairs_.clear();
  }

  inline int size() const
  {
    return static_cast<int>(n_iters_.size());
  }

  double lastBestCost() const
  {
    if (best_costs_.size() == 0)
    {
      return std::numeric_limits<double>::infinity();
    }
    return best_costs_.back();
  }

  void save(const std::string& save_dir) const;
};

extern const std::unordered_map<std::string, RRTCostType> kStrToRRTCostT;
extern const std::unordered_map<RRTCostType, std::string> kRRTCostTToStr;
using RRTCostScalarMap = std::unordered_map<RRTCostType, double>;

template <typename T>
class QuadRRT : public PlannerBase<T>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Point = pcl::PointXYZRGB;
  using PointCloud = pcl::PointCloud<Point>;

  QuadRRT() = delete;
  QuadRRT(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  void plan() override;

  void visualize() override;

  void saveResults() override;

  void resetPlannerState() override;

private:
  bool setPlannerStateCallback(PlanConfig::Request& req,
                               PlanConfig::Response& res) override;

  void clearRRT();
  bool clearRRTPlannerCallback(std_srvs::Empty::Request& req,
                               std_srvs::Empty::Response& res);

  void loadFromYaml(const std::string& param_fn);

  void visualizeRRTVertsEdges() const;
  void visualizeRRTPath() const;

  void extractRRTPathPoses() const;

  void setupRRT();

  inline bool inRange(const Eigen::Vector3d& pt) const
  {
    return (pt.x() > min_x_ && pt.x() < max_x_) &&
           (pt.y() > min_y_ && pt.y() < max_y_) &&
           (pt.z() > min_z_ && pt.z() < max_z_);
  }

  double calValidityResolution() const;

  // visualization related
  std_msgs::ColorRGBA red_, green_, gray_;
  ros::Publisher rrt_vert_pub_;
  ros::Publisher rrt_edge_pub_;

  // service
  ros::ServiceServer clear_rrt_planner_srv_;

  // general configurations
  double robot_radius_;
  double min_x_, max_x_;
  double min_y_, max_y_;
  double min_z_, max_z_;
  double max_range_;

  // ompl stuff
  ob::StateSpacePtr flat_ss_ = nullptr;
  ob::SpaceInformationPtr flat_si_ = nullptr;
  ob::ProblemDefinitionPtr pdef_ = nullptr;
  ob::PlannerPtr planner_ = nullptr;
  ob::PlannerDataPtr planner_data_ = nullptr;
  bool has_valid_solution_ = false;
  bool has_exact_solution_ = false;

  // rrt related
  double rrt_range_ = -1;

  // outer loop iteration control
  double iter_time_sec_ = -1.0;
  int max_n_iter_ = -1;
  int min_n_iter_ = -1;
  double converge_ratio_ = -1.0;
  int consec_lower_than_thresh_cnt_ = 0;
  bool viz_every_outer_iter_ = false;

  // planner state
  Eigen::Vector3d start_pos_;
  Eigen::Vector3d end_pos_;
  double start_yaw_rad_ = 0.0;
  double end_yaw_rad_ = 0.0;
  bool use_esdf_checker_ = false;
  bool use_joint_checker_ = false;

  bool cal_info_from_pc_ = false;
  act_map::InfoMetricType info_metric_type_ = act_map::InfoMetricType::kNone;

  rpg::Pose Tbc_;
  RRTCostScalarMap rrt_cost_to_weight_map_;

  // results
  mutable rpg::PoseVec final_Twb_vec_;
  RRTStats rrt_stats_;
  std::string save_abs_dir_;
};

}

#include "act_map_exp/quad_rrt_impl.h"
