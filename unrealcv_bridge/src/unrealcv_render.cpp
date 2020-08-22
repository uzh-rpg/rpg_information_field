#include "unrealcv_bridge/unrealcv_render.h"

#include <rpg_common/load.h>

#include "unrealcv_bridge/ue_utils.hpp"

namespace unrealcv_bridge
{
void UnrealCVRender::loadUEPoses(const std::string& abs_fn)
{
  Eigen::MatrixXd pose_data;
  rpg_common::load(abs_fn, &pose_data);
  CHECK_EQ(pose_data.cols(), 6);
  for (int ri = 0; ri < pose_data.rows(); ri++)
  {
    ue_poses_.emplace_back(UEPose());
    ue_poses_.back().fromEigen(pose_data.row(ri));
  }
}

void UnrealCVRender::renderImg(const UEPose& uep, cv::Mat* img) const
{
  CHECK(img);
  setCameraDataFromUEPose(uep);
  (*img) = client_->getImage(0);
}

void UnrealCVRender::renderImg(cv::Mat* img) const
{
  CHECK(img);
  (*img) = client_->getImage(0);
}

void UnrealCVRender::renderDepth(const UEPose &uep, const DepthMode& depth_mode,
                                 cv::Mat *depth) const
{
  setCameraDataFromUEPose(uep);
  renderDepth(depth_mode, depth);
}

void UnrealCVRender::renderDepth(const DepthMode& depth_mode,
                                 cv::Mat *depth) const
{
  CHECK(depth);

  cv::Mat ray_depth = client_->getDepth(0);

  if (depth_mode == DepthMode::kZDepth)
  {
    CHECK_GT(this->focal_, 0.0);
    rayDepthToZDepth(ray_depth, this->focal_, depth);
  }
  else if (depth_mode == DepthMode::kRayDepth)
  {
    ray_depth.copyTo(*depth);
  }
  else
  {
    LOG(FATAL) << "Unknown depth mode.";
  }
}

void UnrealCVRender::getSimPose(UEPose *uep) const
{
  client_->getUEPose(0, uep);
}

void UnrealCVRender::getSimTwc(rpg::Pose *Twc) const
{
  UEPose uep;
  client_->getUEPose(0, &uep);
  uep.toTwc(Twc);
}

void UnrealCVRender::setCameraDataFromUEPose(const UEPose &uep) const
{
  CameraData cdata;
  cdata.id = 0u;
  cdata.setUEPose(uep);
  client_->setCamera(cdata);
}

void UnrealCVRender::setCameraGeomery(const int width, const int height,
                                      const float wfov_deg)
{
  //  https://github.com/unrealcv/unrealcv/issues/11
  LOG(FATAL) << "changing camera geometry is not working, "
                "need to change it in the configuration file.";
  cam_width_ = width;
  cam_height_ = height;
  cam_wfov_deg_ = wfov_deg;
  client_->setCameraFOV(wfov_deg);
  this->sleep(1000);
  client_->setCameraSize(width, height);
  this->sleep(1000);
}

void UnrealCVRender::sleep(int ms) const
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

}  // namespace unrealcv_bridge
