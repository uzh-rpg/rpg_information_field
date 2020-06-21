/*
 * ros_params_helper.h
 *
 *  Created on: Feb 22, 2013
 *      Author: cforster
 *
 * from libpointmatcher_ros
 */

#pragma once

#include <string>

#include <ros/ros.h>

namespace rpg_common_ros {

inline
bool hasParam(const std::string& name)
{
  return ros::param::has(name);
}

template<typename T>
T getParam(const std::string& name, const T& defaultValue)
{
  T v;
  if(ros::param::get(name, v))
  {
    ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
    return v;
  }
  else
    ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
  return defaultValue;
}

template<typename T>
T getParam(const std::string& name)
{
  T v;
  if(ros::param::get(name, v))
  {
    ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
    return v;
  }
  else
    ROS_ERROR_STREAM("Cannot find value for parameter: " << name);
  return T();
}

template<typename T>
T param(const ros::NodeHandle& nh, const std::string& name, const T& defaultValue)
{
  if(nh.hasParam(name))
  {
    T v;
    nh.param<T>(name, v, defaultValue);
    ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
    return v;
  }
  ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
  return defaultValue;
}

} // namespace rpg_common_ros

namespace rpg_ros = rpg_common_ros;
