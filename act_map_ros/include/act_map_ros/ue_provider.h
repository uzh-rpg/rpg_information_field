#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>

#include <unrealcv_bridge/unrealcv_render.h>
#include <vi_utils/cam_min.h>

namespace act_map_ros
{
class UEProvider
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Point = pcl::PointXYZI;
  using PointCloud = pcl::PointCloud<Point>;
  using PointRGB = pcl::PointXYZRGB;
  using PointCloudRGB = pcl::PointCloud<PointRGB>;

  UEProvider() = delete;
  UEProvider(const UEProvider&) = delete;
  UEProvider& operator=(const UEProvider&) = delete;

  UEProvider(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  const static std::string kWorldFrame;
  const static std::string kCamFrame;

  virtual ~UEProvider();
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  unrealcv_bridge::UnrealCVRender render_;

  image_transport::ImageTransport it_;
  image_transport::Publisher pub_img_;
  image_transport::Publisher pub_dimg_;

  ros::Publisher pub_sensor_pc_;
  ros::Publisher pub_freespace_pc_;

  ros::Timer pub_timer_;
  void pubCallback(const ros::TimerEvent&);
  void pubTf(ros::Time* time);

  // cameras and bearing vectors
  vi::PinholeCamPtr cam_;
  void initCamera();

  // parameters
  double pub_hz_ = 30.0;
  bool pub_depth_as_img_ = false;
  double max_depth_ = -1;
  bool use_ray_depth_ = false;

  // freespace pointcloud
  bool use_freespace_pc_ = false;
  int n_freespace_pt_per_ray_ = 20;

  // automatic scan
  bool automatic_ = false;
  rpg::PositionVec sample_pos_;
  std::vector<double> sample_yaw_deg_;
  size_t auto_pos_idx_ = 0;
  size_t auto_yaw_idx_ = 0;

  // camera intrinsics
  double focal_ = -1;
  int img_w_ = -1;
  int img_h_ = -1;

};

}
