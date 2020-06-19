//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <iostream>
#include <tuple>

#include <kindr/minimal/common.h>

#include "vi_utils/vi_basic_types.h"

// collection of residual jacobians for visual-inertial navigation
namespace vi_utils
{
// Unless specified otherwise,
// we use the following convention for jacobians
//   * the retraction is done by T * Exp(\epsilon)
//	 * the residual is defind as (predicted - measured),
//   * rotation residuals are expressed in so(3)
// The jacobians are residual w.r.t. infinitesimal update.
// The states are arranges as: position, rotation, velocity
namespace jacobians
{
inline Eigen::Matrix3d inverseRightJacobian(const Eigen::Vector3d& phi)
{
  const double phi_n = phi.norm();
  Eigen::Matrix3d phi_skew = kindr::minimal::skewMatrix(phi);

  return Eigen::Matrix3d::Identity() + 0.5 * phi_skew +
         (1.0 / (phi_n * phi_n) +
          (1.0 + std::cos(phi_n)) / (2.0 * phi_n * std::sin(phi_n))) *
             phi_skew * phi_skew;
}

inline Eigen::Matrix3d rightJacobian(const Eigen::Vector3d& phi)
{
  const double phi_n = phi.norm();
  Eigen::Matrix3d phi_skew = kindr::minimal::skewMatrix(phi);

  return Eigen::Matrix3d::Identity() -
      (1 - std::cos(phi_n)) * phi_skew / (phi_n * phi_n) +
      (phi_n - std::sin(phi_n)) * (phi_skew * phi_skew) / (std::pow(phi_n, 3));
}


inline rpg::Matrix66 dLocal_dGlobal(const rpg::Pose& pose)
{
  rpg::Matrix66 jac_local_global;
  jac_local_global.setZero();
  jac_local_global.block<3, 3>(0, 0) = pose.getRotationMatrix().transpose();
  jac_local_global.block<3, 3>(3, 3) = pose.getRotationMatrix().transpose();
  jac_local_global.block<3, 3>(0, 3) =
      -pose.getRotationMatrix().transpose() *
      kindr::minimal::skewMatrix(pose.getPosition());
  return jac_local_global;
}

// pt_tf = pose * Exp(eps) *  pt; d(pt_tf) / d(eps)
inline rpg::Matrix36 dPoint_dse3(const rpg::Pose& pose, const Eigen::Vector3d& pt)
{
  rpg::Matrix36 jac;
  jac.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  jac.block<3, 3>(0, 3) = (-1.0) * kindr::minimal::skewMatrix(pt);
  return pose.getRotationMatrix() * jac;
}

// pt_tf = Exp(eps) * pose *  pt; d(pt_tf) / d(eps)
inline rpg::Matrix36 dPoint_dse3_leftinc(const rpg::Pose& pose,
                                      const Eigen::Vector3d& pt)
{
  rpg::Matrix36 jac;
  jac.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  jac.block<3, 3>(0, 3) = (-1.0) * kindr::minimal::skewMatrix(pose * pt);
  return jac;
}

inline rpg::Matrix36 dPoint_dse3_org(const Eigen::Vector3d& pt)
{
  rpg::Pose org;
  org.setIdentity();
  return dPoint_dse3(org, pt);
}

// projection jacobian without distortion
inline rpg::Matrix23
dFeature_dPointCam(const double fx, const double fy, const Eigen::Vector3d& p_c)
{
  rpg::Matrix23 jac;
  const double x = p_c(0);
  const double y = p_c(1);
  const double z_inv = 1 / p_c(2);
  const double z_inv_2 = z_inv * z_inv;

  // jac w.r.t. measurement
  jac << fx * z_inv, 0, -fx * x * z_inv_2, 0, fy * z_inv, -fy * y * z_inv_2;

  return jac;
}

// camera jacobians without distortion
// proj(T_c_i * (T_w_i * Exp(eps))^(-1) * p_w)
inline rpg::Matrix26 dFeature_dIMUPose(
    const double fx,
    const double fy,
    const Eigen::Vector3d& p_w,
    const rpg::Pose& T_i_w,
    const rpg::Pose& T_c_i)
{
  Eigen::Vector3d p_c = T_c_i * T_i_w * p_w;

  rpg::Matrix23 du_dpc = dFeature_dPointCam(fx, fy, p_c);
  rpg::Matrix33 dpc_dpi = T_c_i.getRotationMatrix();
  rpg::Matrix36 dpi_deps = - dPoint_dse3_leftinc(T_i_w, p_w);

  return du_dpc * dpc_dpi * dpi_deps;
}

// proj((Exp(eps) * T_w_c)^(-1) * p_w)
inline rpg::Matrix26 dFeature_dIMUPoseGlobal(
    const double fx,
    const double fy,
    const Eigen::Vector3d& p_w,
    const rpg::Pose& T_c_w)
{
  Eigen::Vector3d p_c = T_c_w * p_w;

  rpg::Matrix23 du_dpc = dFeature_dPointCam(fx, fy, p_c);
  rpg::Matrix36 dpc_deps = - dPoint_dse3(T_c_w, p_w);

  return du_dpc * dpc_deps;
}

inline rpg::Matrix23 dFeature_dPointWorld(
    const double fx,
    const double fy,
    const Eigen::Vector3d& p_w,
    const rpg::Pose& T_i_w,
    const rpg::Pose& T_c_i)
{
  rpg::Pose T_c_w = T_c_i * T_i_w;
  Eigen::Vector3d p_c = T_c_i * T_i_w * p_w;

  rpg::Matrix23 du_dpc = dFeature_dPointCam(fx, fy, p_c);
  rpg::Matrix33 dpc_dpw = T_c_w.getRotationMatrix();

  return du_dpc * dpc_dpw;
}

// feature positions w.r.t. camera pose and landmark coordinates in world frame
inline std::tuple<rpg::Matrix26, rpg::Matrix23> dFeature_dStates(
    const double fx,
    const double fy,
    const Eigen::Vector3d& p_w,
    const rpg::Pose& T_i_w,
    const rpg::Pose& T_c_i)
{
  rpg::Pose T_c_w = T_c_i * T_i_w;
  Eigen::Vector3d p_c = T_c_i * T_i_w * p_w;

  rpg::Matrix23 du_dpc = dFeature_dPointCam(fx, fy, p_c);
  rpg::Matrix33 dpc_dpw = T_c_w.getRotationMatrix();
  rpg::Matrix33 dpc_dpi = T_c_i.getRotationMatrix();
  rpg::Matrix36 dpi_deps = - dPoint_dse3_leftinc(T_i_w, p_w);

  return std::make_tuple(du_dpc * dpc_dpi * dpi_deps, du_dpc * dpc_dpw);
}

// feature positions w.r.t. camera pose and landmark coordinates in world frame
inline std::tuple<rpg::Matrix26, rpg::Matrix23> dFeature_dStatesGlobal(
    const double fx,
    const double fy,
    const Eigen::Vector3d& p_w,
    const rpg::Pose& T_i_w,
    const rpg::Pose& T_c_i)
{

  rpg::Matrix26 du_dpose;
  rpg::Matrix23 du_dpw;
  std::tie(du_dpose, du_dpw) = dFeature_dStates(fx, fy, p_w, T_i_w, T_c_i);
  rpg::Matrix66 dlocal_dglobal = dLocal_dGlobal(T_i_w.inverse());

  return std::make_tuple(du_dpose * dlocal_dglobal, du_dpw);
}

// bearing vector representation
inline rpg::Matrix33 dBearing_dPointCam(const Eigen::Vector3d& pc)
{
  const double n = pc.norm();
  rpg::Matrix33 jac = (1 / n) * rpg::Matrix33::Identity() -
      (1 / (n * n * n)) * pc * pc.transpose();
  return jac;
}

// f = normalize( (Exp(eps)*T_w_c)^(-1) * p_w) ; df / deps
inline rpg::Matrix36 dBearing_dIMUPoseGlobal(
    const Eigen::Vector3d& p_w,
    const rpg::Pose& T_c_w)
{
  Eigen::Vector3d p_c = T_c_w * p_w;

  Eigen::Matrix3d df_dpc = dBearing_dPointCam(p_c);
  rpg::Matrix36 dpc_dse3 = -dPoint_dse3(T_c_w, p_w);
  return df_dpc * dpc_dse3;
}

// f = normalize((T_w_i * Exp(eps) * T_i_c)^(-1) * p_w)
inline rpg::Matrix36 dBearing_dIMUPose(
    const Eigen::Vector3d& p_w,
    const rpg::Pose& T_i_w,
    const rpg::Pose& T_c_i)
{
  rpg::Pose T_c_w = T_c_i * T_i_w;
  Eigen::Vector3d p_c = T_c_w * p_w;

  Eigen::Matrix3d df_dpc = dBearing_dPointCam(p_c);
  Eigen::Matrix3d dpc_dpi = T_c_i.getRotationMatrix();
  rpg::Matrix36 dpi_dse3 = -dPoint_dse3_leftinc(T_i_w, p_w);

  return df_dpc * dpc_dpi * dpi_dse3;
}

// imu factor jacobians without bias
// For IMU, all the states are in the navigation frame,
// which can be the coordinate of the first frame.

// imu position integration w.r.t. previous states, current states, and gravity
inline std::tuple<rpg::Matrix39, rpg::Matrix39, rpg::Matrix33>
dPreintegratedPosition_dStates(
    const rpg::Pose& pose_i,
    const rpg::Pose& pose_j,
    const Eigen::Vector3d& v_i,
    const Eigen::Vector3d& g,
    const double tij)
{

  const Eigen::Vector3d p_i = pose_i.getPosition();
  const Eigen::Vector3d p_j = pose_j.getPosition();
  const Eigen::Matrix3d R_i = pose_i.getRotationMatrix();
  const Eigen::Matrix3d R_j = pose_j.getRotationMatrix();
  // previous states
  rpg::Matrix39 jac_i;
  jac_i.block<3, 3>(0, 0) = (-1.0) * Eigen::Matrix3d::Identity();
  jac_i.block<3, 3>(0, 3) = kindr::minimal::skewMatrix(
      R_i.transpose() * (p_j - p_i - v_i * tij - 0.5 * g * tij * tij));
  jac_i.block<3, 3>(0, 6) = (-1.0) * R_i.transpose() * tij;

  // current states
  rpg::Matrix39 jac_j;
  jac_j.block<3, 3>(0, 0) = R_i.transpose() * R_j;
  jac_j.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
  jac_j.block<3, 3>(0, 6) = Eigen::Matrix3d::Zero();

  // g
  Eigen::Matrix3d jac_g = (-0.5) * R_i.transpose() * tij * tij;

  return std::make_tuple(jac_i, jac_j, jac_g);
}

inline std::tuple<rpg::Matrix39, rpg::Matrix39, rpg::Matrix33>
dPreintegratedPosition_dStatesGlobal(
    const rpg::Pose& pose_i,
    const rpg::Pose& pose_j,
    const Eigen::Vector3d& v_i,
    const Eigen::Vector3d& g,
    const double tij)
{
  rpg::Matrix39 jac_i;
  rpg::Matrix39 jac_j;
  rpg::Matrix33 jac_g;
  std::tie(jac_i, jac_j, jac_g) =
      dPreintegratedPosition_dStates(pose_i, pose_j, v_i, g, tij);
  rpg::Matrix66 dl_dg_i = dLocal_dGlobal(pose_i);
  rpg::Matrix66 dl_dg_j = dLocal_dGlobal(pose_j);
  jac_i.block<3, 6>(0, 0) = jac_i.block<3, 6>(0, 0) * dl_dg_i;
  jac_j.block<3, 6>(0, 0) = jac_j.block<3, 6>(0, 0) * dl_dg_j;

  return std::make_tuple(jac_i, jac_j, jac_g);
}

// imu rotation itegration w.r.t. previous states, current states, and gravity
inline std::tuple<rpg::Matrix39, rpg::Matrix39, rpg::Matrix33>
dPreintegratedRotation_dStates(
    const Eigen::Matrix3d& R_i, const Eigen::Matrix3d& R_j)
{
  // since we are considering jacobian at the correct estimation
  // the right jacobian is simply identity
  rpg::Matrix39 jac_i;
  jac_i.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
  jac_i.block<3, 3>(0, 3) = (-1.0) * R_j.transpose() * R_i;
  jac_i.block<3, 3>(0, 6) = Eigen::Matrix3d::Zero();

  rpg::Matrix39 jac_j;
  jac_j.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
  jac_j.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
  jac_j.block<3, 3>(0, 6) = Eigen::Matrix3d::Zero();

  Eigen::Matrix3d jac_g = Eigen::Matrix3d::Zero();

  return std::make_tuple(jac_i, jac_j, jac_g);
}

inline std::tuple<rpg::Matrix39, rpg::Matrix39, rpg::Matrix33>
dPreintegratedRotation_dStatesGlobal(
    const rpg::Pose& pose_i, const rpg::Pose& pose_j)
{
  rpg::Matrix39 jac_i;
  rpg::Matrix39 jac_j;
  rpg::Matrix33 jac_g;
  std::tie(jac_i, jac_j, jac_g) =
      dPreintegratedRotation_dStates(
        pose_i.getRotationMatrix(), pose_j.getRotationMatrix());
  rpg::Matrix66 dl_dg_i = dLocal_dGlobal(pose_i);
  rpg::Matrix66 dl_dg_j = dLocal_dGlobal(pose_j);
  jac_i.block<3, 6>(0, 0) = jac_i.block<3, 6>(0, 0) * dl_dg_i;
  jac_j.block<3, 6>(0, 0) = jac_j.block<3, 6>(0, 0) * dl_dg_j;

  return std::make_tuple(jac_i, jac_j, jac_g);
}

// imu velocity integration w.r.t. previous states, current states, and gravity
inline std::tuple<rpg::Matrix39, rpg::Matrix39, rpg::Matrix33>
dPreintegratedVelocity_dStates(
    const Eigen::Matrix3d& R_i,
    const Eigen::Vector3d& v_i,
    const Eigen::Vector3d& v_j,
    const Eigen::Vector3d& g,
    const double tij)
{
  // previous stats
  rpg::Matrix39 jac_i;
  jac_i.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
  jac_i.block<3, 3>(0, 3) =
      kindr::minimal::skewMatrix(R_i.transpose() * (v_j - v_i - g * tij));
  jac_i.block<3, 3>(0, 6) = (-1.0) * R_i.transpose();

  // current states
  rpg::Matrix39 jac_j;
  jac_j.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
  jac_j.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
  jac_j.block<3, 3>(0, 6) = R_i.transpose();

  // g
  Eigen::Matrix3d jac_g = (-1.0) * R_i.transpose() * tij;

  return std::make_tuple(jac_i, jac_j, jac_g);
}

inline std::tuple<rpg::Matrix39, rpg::Matrix39, rpg::Matrix33>
dPreintegratedVelocity_dStatesGlobal(
    const rpg::Pose& pose_i,
    const rpg::Pose& pose_j,
    const Eigen::Vector3d& v_i,
    const Eigen::Vector3d& v_j,
    const Eigen::Vector3d& g,
    const double tij)
{
  rpg::Matrix39 jac_i;
  rpg::Matrix39 jac_j;
  rpg::Matrix33 jac_g;
  std::tie(jac_i, jac_j, jac_g) =
      dPreintegratedVelocity_dStates(
        pose_i.getRotationMatrix(), v_i, v_j, g, tij);

  rpg::Matrix66 dl_dg_i = dLocal_dGlobal(pose_i);
  rpg::Matrix66 dl_dg_j = dLocal_dGlobal(pose_j);
  jac_i.block<3, 6>(0, 0) = jac_i.block<3, 6>(0, 0) * dl_dg_i;
  jac_j.block<3, 6>(0, 0) = jac_j.block<3, 6>(0, 0) * dl_dg_j;

  return std::make_tuple(jac_i, jac_j, jac_g);
}

}
}
