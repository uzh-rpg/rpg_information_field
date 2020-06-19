//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <fstream>

#include <Eigen/Core>
#include <rpg_common/pose.h>

namespace vi_utils
{
struct States
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  States()
    : time_ns(-1)
    , T_0_cur()
    , v_cur(Eigen::Vector3d(0.0, 0.0, 0.0))
    , acc_bias(Eigen::Vector3d(0.0, 0.0, 0.0))
    , gyr_bias(Eigen::Vector3d(0.0, 0.0, 0.0))
  {
  }

  int64_t time_ns;
  rpg::Pose T_0_cur;
  Eigen::Vector3d v_cur;
  Eigen::Vector3d acc_bias;
  Eigen::Vector3d gyr_bias;

  friend std::ofstream& operator<<(std::ofstream& out, const States& rhs);
  static void toEigen(const std::vector<States>& states, Eigen::MatrixXd* mat);
  static void getTimeSecVec(const std::vector<States>& states,
                            std::vector<double>* sec_vec);

  static void save(const std::vector<States>& states,
                   const std::string& abs_save);
  static void load(const std::string& abs_load, std::vector<States>* states,
                   const char delmi=' ', const bool timestamp_ns=false);

  // utility functions
  static void disturbStates(const std::vector<States>& pre_states,
                            const double trans_dist,
                            const double angle_rad,
                            const double vel_mag,
                            std::vector<States>* dist_states);
};
using StatesVec = std::vector<States>;

struct StatesError
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double att_err_deg;  // angle of angular-axis
  Eigen::Vector3d pos_err;
  Eigen::Vector3d vel_err;

  enum class AlignType
  {
    kNaive,     // directly calculate the difference
    kAlignSE3,  // align the first frames by SE3
    kAlignYaw   // align position and yaw
  };
  static void calStatesError(const States& states1,
                             const States states2,
                             StatesError* states_err);

  static std::tuple<double, double, double>
  calIMUStatesRMSE(const StatesVec& gt,
                   const StatesVec& meas,
                   const AlignType e_type);
};
using StatesErrorVec = std::vector<StatesError>;

}
namespace vi
{
using States = vi_utils::States;
using StatesVec = vi_utils::StatesVec;
using StatesError = vi_utils::StatesError;
using StatesErrorVec = vi_utils::StatesErrorVec;
}
