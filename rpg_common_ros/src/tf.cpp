//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "rpg_common_ros/tf.h"

#include <memory>

#include <glog/logging.h>
#include <minkindr_conversions/kindr_tf.h>
#include <ros/time.h>
#include <tf/transform_listener.h>

namespace rpg_common_ros {
namespace tf {

::tf::TransformListener& getListener()
{
  static std::unique_ptr<::tf::TransformListener> listener;
  if (!listener)
  {
    listener.reset(new ::tf::TransformListener);
  }
  return *listener;
}

void initListener()
{
  getListener();
}

rpg::Pose get_T_A_B(const std::string& A_name, const std::string& B_name)
{
  return get_T_A_B(A_name, B_name, ros::Time::now());
}

rpg::Pose get_T_A_B(const std::string& A_name, const std::string& B_name,
                    const ros::Time& time)
{
  rpg::Pose T_A_B;
  CHECK(get_T_A_B(A_name, B_name, time, &T_A_B));
  return T_A_B;
}

bool get_T_A_B(
    const std::string& A_name, const std::string& B_name, rpg::Pose* T_A_B)
{
  return get_T_A_B(A_name, B_name, ros::Time::now(), T_A_B);
}

bool get_T_A_B(
    const std::string& A_name, const std::string& B_name,
    const ros::Time& time, rpg::Pose* T_A_B)
{
  CHECK_NOTNULL(T_A_B);
  try
  {
    ::tf::StampedTransform T_A_B_tf;
    getListener().lookupTransform(A_name, B_name, time, T_A_B_tf);
    ::tf::poseTFToKindr(T_A_B_tf, T_A_B);
    return true;
  }
  catch (const ::tf::TransformException& e)
  {
    LOG(WARNING) << "TF listening failed: " << e.what();
    return false;
  }
}

}  // namespace tf
}  // namespace rpg_common_ros
namespace rpg_ros = rpg_common_ros;
