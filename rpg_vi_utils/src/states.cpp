//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "vi_utils/states.h"

#include <ctime>

#include <rpg_common/save.h>
#include <rpg_common/load.h>
#include <rpg_common/fs.h>

namespace vi_utils
{
void States::toEigen(const std::vector<States>& states, Eigen::MatrixXd* mat)
{
  CHECK_NOTNULL(mat);
  CHECK(!states.empty());
  int num_states = states.size();
  mat->resize(num_states, 17);

  for (int i = 0; i < num_states; i++)
  {
    const States& s = states[i];
    (*mat)(i, 0) = static_cast<double>(s.time_ns) / 1e9;
    (*mat)(i, 1) = (s.T_0_cur.getPosition())(0);
    (*mat)(i, 2) = (s.T_0_cur.getPosition())(1);
    (*mat)(i, 3) = (s.T_0_cur.getPosition())(2);
    (*mat)(i, 4) = s.T_0_cur.getRotation().w();
    (*mat)(i, 5) = s.T_0_cur.getRotation().x();
    (*mat)(i, 6) = s.T_0_cur.getRotation().y();
    (*mat)(i, 7) = s.T_0_cur.getRotation().z();
    (*mat)(i, 8) = s.v_cur[0];
    (*mat)(i, 9) = s.v_cur[1];
    (*mat)(i, 10) = s.v_cur[2];

    (*mat)(i, 11) = s.acc_bias[0];
    (*mat)(i, 12) = s.acc_bias[1];
    (*mat)(i, 13) = s.acc_bias[2];

    (*mat)(i, 14) = s.gyr_bias[0];
    (*mat)(i, 15) = s.gyr_bias[1];
    (*mat)(i, 16) = s.gyr_bias[2];
  }
}

std::ofstream& operator<<(std::ofstream& out, const States& rhs)
{
  const std::string delim(" ");
  out << rhs.T_0_cur.getPosition().x() << delim << rhs.T_0_cur.getPosition().y()
      << delim << rhs.T_0_cur.getPosition().z() << delim
      << rhs.T_0_cur.getRotation().w() << delim << rhs.T_0_cur.getRotation().x()
      << delim << rhs.T_0_cur.getRotation().y() << delim
      << rhs.T_0_cur.getRotation().z() << delim << rhs.v_cur.x() << delim
      << rhs.v_cur.y() << delim << rhs.v_cur.z() << rhs.acc_bias[0] << delim
      << rhs.acc_bias[1] << delim << rhs.acc_bias[2] << delim << rhs.gyr_bias[0]
      << delim << rhs.gyr_bias[1] << delim << rhs.gyr_bias[2];

  return out;
}

void States::getTimeSecVec(const std::vector<States>& states,
                           std::vector<double>* sec_vec)
{
  CHECK_NOTNULL(sec_vec);
  sec_vec->resize(states.size());
  for (size_t i = 0; i < states.size(); i++)
  {
    (*sec_vec)[i] = static_cast<double>(states[i].time_ns) / 1e9;
  }
}

void States::save(const StatesVec& states, const std::string& abs_save)
{
  Eigen::MatrixXd mat;
  States::toEigen(states, &mat);
  rpg::save(abs_save, mat);
}

void States::load(const std::string& abs_load, StatesVec* states,
                  const char delim, const bool timestamp_ns)
{
  CHECK_NOTNULL(states);
  CHECK(rpg::fs::fileExists(abs_load));

  Eigen::MatrixXd mat;
  rpg::load(abs_load, &mat, delim);

  int n_states = mat.rows();
  states->resize(n_states);

  CHECK(mat.cols() == 8 || mat.cols() == 17);

  for (int i = 0; i < n_states; i++)
  {
    States& s = (*states)[i];
    if (timestamp_ns)
    {
      s.time_ns = static_cast<int64_t>(mat(i, 0));
    }
    else
    {
      s.time_ns = static_cast<int64_t>(mat(i, 0) * 1e9);
    }
    s.T_0_cur =
        rpg::Pose(rpg::Position(mat(i, 1), mat(i, 2), mat(i, 3)),
                  rpg::Rotation(mat(i, 4), mat(i, 5), mat(i, 6), mat(i, 7)));
    if (mat.cols() == 17)
    {
      s.v_cur = Eigen::Vector3d(mat(i, 8), mat(i, 9), mat(i, 10));
      s.acc_bias = Eigen::Vector3d(mat(i, 11), mat(i, 12), mat(i, 13));
      s.gyr_bias = Eigen::Vector3d(mat(i, 14), mat(i, 15), mat(i, 16));
    }
  }
}

void States::disturbStates(const StatesVec& pre_states,
                           const double trans_dist,
                           const double angle_deg,
                           const double vel_mag,
                           StatesVec* dist_states)
{
  CHECK_NOTNULL(dist_states);
  std::srand(std::time(0));
  dist_states->resize(pre_states.size());
  for (size_t i = 0; i < pre_states.size(); i++)
  {
    rpg::Pose T_dist;
    T_dist.setRandom(trans_dist, angle_deg * 3.14 / 180.0);
    (*dist_states)[i].time_ns = pre_states[i].time_ns;
    (*dist_states)[i].T_0_cur = pre_states[i].T_0_cur * T_dist;
    (*dist_states)[i].v_cur =
        pre_states[i].v_cur + vel_mag * Eigen::Vector3d::Random();
    (*dist_states)[i].acc_bias = Eigen::Vector3d::Zero();
    (*dist_states)[i].gyr_bias = Eigen::Vector3d::Zero();
  }
}

void StatesError::calStatesError(const States& states1,
                                 const States states2,
                                 vi::StatesError* states_err)
{
  CHECK_NOTNULL(states_err);

  rpg::Pose tf_err = states1.T_0_cur.inverse() * states2.T_0_cur;
  Eigen::AngleAxis<double> angle_axis_err(tf_err.getRotationMatrix());
  states_err->att_err_deg = angle_axis_err.angle() / M_PI * 180.0;
  states_err->pos_err = tf_err.getPosition();
  states_err->vel_err = states1.v_cur - states2.v_cur;
}

std::tuple<double, double, double> StatesError::calIMUStatesRMSE(
    const StatesVec& gt, const StatesVec& meas, const AlignType e_type)
{
  CHECK_EQ(gt.size(), meas.size());
  size_t num_states = gt.size();

  vi::StatesVec aligned_meas;
  // align the trajectory if necessary
  if (e_type == AlignType::kAlignSE3)
  {
    LOG(FATAL) << "Not implemented!";
  }
  else if (e_type == AlignType::kAlignYaw)
  {
    // NOTE: the euler angles in Eigen are rotating euler...
    // therefore there is no way to guarantee that
    // the following rotations have no component in the global z direction.
    // We would need static euler angles in this case.
    LOG(FATAL) << "Not implemented!";
  }
  else if (e_type == AlignType::kNaive)
  {
    aligned_meas = meas;
  }
  else
  {
    LOG(FATAL) << "Unknown error type.";
  }

  vi::StatesErrorVec err_vec(gt.size());
  for (size_t i = 0; i < gt.size(); ++i)
  {
    calStatesError(gt[i], aligned_meas[i], &err_vec[i]);
  }

  double att_deg_ss = 0.0;
  double pos_ss = 0.0;
  double vel_ss = 0.0;

  for (size_t i = 0; i < num_states; ++i)
  {
    att_deg_ss += std::pow(err_vec[i].att_err_deg, 2);
    pos_ss += err_vec[i].pos_err.squaredNorm();
    vel_ss += err_vec[i].vel_err.squaredNorm();
  }

  return std::make_tuple(std::sqrt(att_deg_ss / num_states),
                         std::sqrt(pos_ss / num_states),
                         std::sqrt(vel_ss / num_states));
}

namespace utils
{
}
}
