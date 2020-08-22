#pragma once

#include <cmath>

#include <rpg_common/pose.h>

namespace esim
{
/**
 * https://github.com/EpicGames/UnrealEngine/blob/dbced2dd59f9f5dfef1d7786fd67ad2970adf95f/Engine/Source/Runtime/Core/Public/Math/Rotator.h#L580
 * Helper function for eulerFromQuatSingularityTest, angles are expected to be
 *given in degrees
 **/
inline double clampAxis(double angle)
{
  // returns angle in the range (-360,360)
  angle = std::fmod(angle, 360.f);

  if (angle < 0.0)
  {
    // shift to [0,360) range
    angle += 360.0;
  }

  return angle;
}

/**
 * https://github.com/EpicGames/UnrealEngine/blob/dbced2dd59f9f5dfef1d7786fd67ad2970adf95f/Engine/Source/Runtime/Core/Public/Math/Rotator.h#L595$
 * Helper function for eulerFromQuatSingularityTest, angles are expected to be
 *given in degrees
 **/
inline double normalizeAxis(double angle)
{
  angle = clampAxis(angle);
  if (angle > 180.0)
  {
    // shift to (-180,180]
    angle -= 360.0;
  }

  return angle;
}

/**
 *
 * https://github.com/EpicGames/UnrealEngine/blob/f794321ffcad597c6232bc706304c0c9b4e154b2/Engine/Source/Runtime/Core/Private/Math/UnrealMath.cpp#L540
 * Quaternion given in (x,y,z,w) representation
 **/
inline void quaternionToEulerUnrealEngine(const rpg_common::Rotation& q,
                                          double& yaw, double& pitch,
                                          double& roll)
{
  const double X = q.x();
  const double Y = q.y();
  const double Z = q.z();
  const double W = q.w();

  const double SingularityTest = Z * X - W * Y;
  const double YawY = 2.0 * (W * Z + X * Y);
  const double YawX = (1.0 - 2.0 * (Y * Y + Z * Z));

  // reference
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/

  // this value was found from experience, the above websites recommend
  // different values but that isn't the case for us, so I went through
  // different testing, and finally found the case where both of world lives
  // happily.
  const double SINGULARITY_THRESHOLD = 0.4999995;
  const double RAD_TO_DEG = (180.0) / M_PI;

  if (SingularityTest < -SINGULARITY_THRESHOLD)
  {
    pitch = -90.;
    yaw = std::atan2(YawY, YawX) * RAD_TO_DEG;
    roll = normalizeAxis(-yaw - (2.0 * std::atan2(X, W) * RAD_TO_DEG));
  }
  else if (SingularityTest > SINGULARITY_THRESHOLD)
  {
    pitch = 90.;
    yaw = std::atan2(YawY, YawX) * RAD_TO_DEG;
    roll = normalizeAxis(yaw - (2.0 * std::atan2(X, W) * RAD_TO_DEG));
  }
  else
  {
    pitch = std::asin(2.0 * (SingularityTest)) * RAD_TO_DEG;
    yaw = std::atan2(YawY, YawX) * RAD_TO_DEG;
    roll = std::atan2(-2.0 * (W * X + Y * Z), (1.0 - 2.0 * (X * X + Y * Y))) *
           RAD_TO_DEG;
  }
}

// to keep consistent with the above convention
inline void unrealEulerToQuaternion(const double pitch_deg,
                                    const double yaw_deg, const double roll_deg,
                                    rpg::Rotation* rot)
{
  constexpr double kDegToRad = M_PI / 180.0;
  const double roll_rad = -kDegToRad * roll_deg;
  const double pitch_rad = -kDegToRad * pitch_deg;
  const double yaw_rad = kDegToRad * yaw_deg;

  rpg::Rotation rot_roll(Eigen::Vector3d(roll_rad, 0.0, 0.0));
  rpg::Rotation rot_pitch(Eigen::Vector3d(0.0, pitch_rad, 0.0));
  rpg::Rotation rot_yaw(Eigen::Vector3d(0.0, 0.0, yaw_rad));
  (*rot) = rot_yaw * rot_pitch * rot_roll;
}

}  // namespace esim
