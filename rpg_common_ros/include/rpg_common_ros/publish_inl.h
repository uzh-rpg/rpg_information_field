#pragma once

namespace rpg_common_ros {

inline void publishTf(const Eigen::Matrix4d& T_A_B, const std::string& A_name,
                      const std::string& B_name)
{
  publishTf(T_A_B, ros::Time::now(), A_name, B_name);
}
inline void publishTf(const rpg::Pose& T_A_B, const ros::Time& ros_time,
                      const std::string& A_name, const std::string& B_name)
{
  publishTf(T_A_B.getTransformationMatrix(), ros_time, A_name, B_name);
}
inline void publishTf(const rpg::Pose& T_A_B, const std::string& A_name,
        const std::string& B_name)
{
  publishTf(T_A_B.getTransformationMatrix(), A_name, B_name);
}

}  // namespace rpg_common_ros
namespace rpg_ros = rpg_common_ros;
