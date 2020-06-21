#pragma once

#include <Eigen/Dense>
#include <glog/logging.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <rpg_common/pose.h>
#include <std_msgs/Header.h>

namespace rpg_common_ros {

namespace publish_internal {

// Create a static publisher on-demand (for temporary use only!).
template <typename MessageType>
ros::Publisher& getPublisher(const std::string& topic_name);

// Non-const; allows to change this. Default is "world".
extern std::string world_frame;

template <class ContainerAllocator>
void setDefaultHeader(std_msgs::Header_<ContainerAllocator>* header)
{
  CHECK_NOTNULL(header);
  header->frame_id = world_frame;
  header->stamp = ros::Time::now();
}
}  // namespace publish_internal

template <typename ... Type>
void publish(const std::string& topic, const Type& ... objects);

void publishTf(const Eigen::Matrix4d& T_A_B, const ros::Time& ros_time,
        const std::string& A_name, const std::string& B_name);
// Time = now:
inline void publishTf(const Eigen::Matrix4d& T_A_B, const std::string& A_name,
                      const std::string& B_name);
inline void publishTf(const rpg::Pose& T_A_B, const ros::Time& ros_time,
                      const std::string& A_name, const std::string& B_name);
inline void publishTf(const rpg::Pose& T_A_B, const std::string& A_name,
                      const std::string& B_name);

}  // namespace rpg_common_ros
namespace rpg_ros = rpg_common_ros;

#include "rpg_common_ros/publish_inl.h"
