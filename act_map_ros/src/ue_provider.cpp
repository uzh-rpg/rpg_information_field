#include "act_map_ros/ue_provider.h"

#include <pcl_conversions/pcl_conversions.h>

#include <rpg_common_ros/params_helper.h>
#include <rpg_common_ros/publish.h>
#include <cv_bridge/cv_bridge.h>
#include <act_map/sampler.h>
#include <vi_utils/common_utils.h>

namespace act_map_ros
{
const std::string UEProvider::kWorldFrame = "world";
const std::string UEProvider::kCamFrame = "cam";

UEProvider::~UEProvider()
{
}

UEProvider::UEProvider(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh)
  , pnh_(pnh)
  , render_()
  , it_(pnh_)
  , pub_img_(it_.advertise("img", 5))
  , pub_dimg_(it_.advertise("depth", 5))
  , pub_sensor_pc_(pnh_.advertise<UEProvider::PointCloud>("sensor_pc", 5))
  , pub_freespace_pc_(pnh_.advertise<UEProvider::PointCloud>("freespace_pc", 5))
{
  pub_hz_ = rpg_common_ros::param<double>(pnh_, "pub_hz", 10);

  focal_ = rpg_common_ros::param<double>(pnh_, "focal", -1.0);
  CHECK_GT(focal_, 0);
  img_h_ = rpg_common_ros::param<int>(pnh_, "img_h", -1);
  CHECK_GT(img_h_, 0);
  img_w_ = rpg_common_ros::param<int>(pnh_, "img_w", -1);
  CHECK_GT(img_w_, 0);
  initCamera();

  pub_depth_as_img_ = rpg_common_ros::param(pnh, "pub_depth_as_img", false);
  max_depth_ = rpg_common_ros::param<double>(pnh, "max_depth", -1.0);
  use_ray_depth_ = rpg_common_ros::param(pnh, "use_ray_depth", false);

  pub_timer_ = pnh_.createTimer(ros::Duration(1.0 / pub_hz_),
                                &UEProvider::pubCallback, this);

  use_freespace_pc_ = rpg_common_ros::param(pnh, "use_freespace_pc", false);
  n_freespace_pt_per_ray_ =
      rpg_common_ros::param<int>(pnh, "freespace_pc_n_step", 20);

  automatic_ = rpg_common_ros::param(pnh, "automatic_scan", false);
  if (automatic_)
  {
    VLOG(1) << "Will automatically scan the map.";
    double auto_pos_step = rpg_ros::param(pnh, "auto_pos_step", 100);
    CHECK_GT(auto_pos_step, 0.0);
    double auto_yaw_step_deg = rpg_ros::param(pnh, "auto_yaw_step_deg", 20.0);
    CHECK_GT(auto_yaw_step_deg, 0.0);
    std::vector<double> ranges;
    pnh_.getParam("auto_scan_ranges", ranges);
    act_map::utils::generateUniformPointsWithin(auto_pos_step, ranges,
                                                &sample_pos_);
    vi_utils::linspace(0.0, 359.0, auto_yaw_step_deg, &sample_yaw_deg_);
    VLOG(1) << "Sampled " << sample_pos_.size() << " positions and "
            << sample_yaw_deg_.size() << " yaw angles for automatic scan.";
    auto_pos_idx_ = 0;
    auto_yaw_idx_ = 0;
  }

  render_.focal() = focal_;
}

void UEProvider::initCamera()
{
  rpg::Pose Tbc;
  Tbc.setIdentity();

  std::vector<double> geo_params{ focal_,
                                  focal_,
                                  0.5 * static_cast<double>(img_w_) - 0.5,
                                  0.5 * static_cast<double>(img_h_) - 0.5,
                                  static_cast<double>(img_w_),
                                  static_cast<double>(img_h_) };
  cam_.reset(new vi::PinholeCam(geo_params, Tbc));
  VLOG(1) << "init camera " << *cam_;

  cam_->computeBearingVectors();
}

void UEProvider::pubCallback(const ros::TimerEvent&)
{
  if (automatic_)
  {
    rpg::Pose Twc;
    Twc.setIdentity();
    Twc.getPosition() = sample_pos_[auto_pos_idx_];
    unrealcv_bridge::UEPose uep;
    unrealcv_bridge::TwcToUEPose(Twc, &uep);
    uep.pitch = 0;
    uep.roll = 0;
    uep.yaw = sample_yaw_deg_[auto_yaw_idx_];

    render_.setCameraDataFromUEPose(uep);

    auto_yaw_idx_++;
    if (auto_yaw_idx_ == sample_yaw_deg_.size())
    {
      auto_yaw_idx_ = 0;
      auto_pos_idx_++;
      VLOG(1) << "Starting scan for position " << auto_pos_idx_ + 1
              << " out of " << sample_pos_.size();
    }
    if (auto_pos_idx_ == sample_pos_.size())
    {
      VLOG(1) << "Autoscan finished!";
      auto_pos_idx_ = 0;
      automatic_ = false;
    }
  }

  ros::Time time;
  // image
  pubTf(&time);
  ros::Time img_time = time;
  cv::Mat img;
  render_.renderImg(&img);
  CHECK_EQ(img.rows, img_h_);
  CHECK_EQ(img.cols, img_w_);

  // depth
  pubTf(&time);
  cv::Mat dimg;
  unrealcv_bridge::DepthMode dm = use_ray_depth_ ?
                                      unrealcv_bridge::DepthMode::kRayDepth :
                                      unrealcv_bridge::DepthMode::kZDepth;
  ros::Time depth_time = time;
  render_.renderDepth(dm, &dimg);
  CHECK_EQ(dimg.rows, img_h_);
  CHECK_EQ(dimg.cols, img_w_);

  // end tf
  pubTf(&time);

  // clean spurious depth values
  cv::threshold(dimg, dimg, 0.0, 0.0, cv::THRESH_TOZERO);

  // publish image
  if (pub_img_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage img_msg;
    img_msg.header.stamp = img_time;
    img_msg.header.frame_id = "cam";
    img_msg.image = img;
    img_msg.encoding = sensor_msgs::image_encodings::BGR8;
    pub_img_.publish(img_msg.toImageMsg());
  }

  // publish pointcloud
  if (pub_sensor_pc_.getNumSubscribers() > 0)
  {
    PointCloudRGB sensor_pc;
    pcl_conversions::toPCL(depth_time, sensor_pc.header.stamp);
    sensor_pc.header.frame_id = kCamFrame;
    sensor_pc.reserve(static_cast<size_t>(cam_->numBearings()));

    for (int ci = 0; ci < img_w_; ci++)
    {
      for (int ri = 0; ri < img_h_; ri++)
      {
        PointRGB pt;
        float d = dimg.at<float>(ri, ci);
        const Eigen::Ref<const Eigen::Vector3d> f =
            cam_->getBearingAtPixel(ci, ri);
        cv::Vec3b& color = img.at<cv::Vec3b>(ri, ci);
        if (static_cast<double>(d) < max_depth_)
        {
          pt.x = static_cast<float>(f(0)) * d;
          pt.y = static_cast<float>(f(1)) * d;
          pt.z = d;
          pt.b = color[0];
          pt.g = color[1];
          pt.r = color[2];
          sensor_pc.push_back(pt);
        }
      }
    }
    pub_sensor_pc_.publish(sensor_pc);
  }

  // freespace pointcloud
  if (use_freespace_pc_ && pub_freespace_pc_.getNumSubscribers() > 0)
  {
    PointCloud fs_pc;
    pcl_conversions::toPCL(depth_time, fs_pc.header.stamp);
    fs_pc.header.frame_id = kCamFrame;
    fs_pc.reserve(
        static_cast<size_t>(cam_->numBearings() * n_freespace_pt_per_ray_));
    constexpr int skip = 10;
    for (int ci = 0; ci < img_w_; ci += skip)
    {
      for (int ri = 0; ri < img_h_; ri += skip)
      {
        Point pt;
        float actual_d = dimg.at<float>(ri, ci);
        double cur_d = static_cast<double>(actual_d) > max_depth_ ?
                           max_depth_ :
                           static_cast<double>(actual_d);
        const double dstep = cur_d / n_freespace_pt_per_ray_;
        const Eigen::Ref<const Eigen::Vector3d> f =
            cam_->getBearingAtPixel(ci, ri);
        for (int fs_i = 0; fs_i < n_freespace_pt_per_ray_; fs_i++)
        {
          const double d = dstep * fs_i;
          pt.x = static_cast<float>(f(0) * d);
          pt.y = static_cast<float>(f(1) * d);
          pt.z = static_cast<float>(d);
          fs_pc.push_back(pt);
        }
      }
    }
    pub_freespace_pc_.publish(fs_pc);
  }

  // visualize depth image
  if (pub_depth_as_img_ && pub_dimg_.getNumSubscribers() > 0)
  {
    if (max_depth_ > 0)
    {
      cv::threshold(dimg, dimg, max_depth_, max_depth_, cv::THRESH_TRUNC);
    }
    cv_bridge::CvImage dimg_msg;
    dimg_msg.header.stamp = depth_time;
    dimg_msg.header.frame_id = "depth";
    dimg.convertTo(dimg, CV_32FC1);
    dimg *= (1.0 / max_depth_);
    dimg_msg.image = dimg;
    dimg_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    pub_dimg_.publish(dimg_msg.toImageMsg());
  }
}

void UEProvider::pubTf(ros::Time* time)
{
  unrealcv_bridge::UEPose uep;
  rpg::Pose Twc;

  (*time) = ros::Time::now();

  render_.getSimPose(&uep);
  uep.toTwc(&Twc);
  rpg_ros::publishTf(Twc, *time, kWorldFrame, kCamFrame);
}

}  // namespace act_map_ros
