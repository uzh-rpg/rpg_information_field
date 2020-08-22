#pragma once

#include "unrealcv_bridge/unrealcv_client.hpp"
#include "unrealcv_bridge/ue_utils.hpp"

namespace unrealcv_bridge
{
enum class DepthMode
{
  kRayDepth,
  kZDepth
};

class UnrealCVRender
{
public:
  UnrealCVRender() : client_(), ue_poses_()
  {
    LOG(WARNING) << "Currently the camera geometry can only be set via "
                    "UnrealCV config.";
    client_.reset(new UnrealCvClient("localhost", "9000"));
  }

  UnrealCVRender(const int width, const int height, const float wfov_deg)
    : UnrealCVRender()
  {
    setCameraGeomery(width, height, wfov_deg);
  }

  virtual ~UnrealCVRender()
  {
  }

  inline void disconnect()
  {
    client_.reset();
  }

  inline void connect()
  {
    if (!client_)
    {
      client_.reset(new UnrealCvClient("localhost", "9000"));
    }
  }

  void loadUEPoses(const std::string& abs_fn);

  inline size_t numPoses() const
  {
    return ue_poses_.size();
  }

  UEPose getUEPoses(const size_t idx) const
  {
    return ue_poses_[idx];
  }

  void renderImg(const UEPose& uep, cv::Mat* img) const;

  void renderImg(cv::Mat* img) const;

  void renderDepth(const UEPose& uep, const DepthMode& depth_mode,
                   cv::Mat* depth) const;

  void renderDepth(const DepthMode& depth_mode, cv::Mat* depth) const;

  void getSimPose(UEPose* uep) const;

  void getSimTwc(rpg::Pose* Twc) const;

  void setCameraGeomery(const int width, const int height,
                        const float wfov_deg);

  void sleep(int ms) const;

  inline float& focal()
  {
    return focal_;
  }

  void setCameraDataFromUEPose(const UEPose& uep) const;

private:

  UEClientPtr client_;

  std::vector<UEPose> ue_poses_;

  int cam_width_ = -1;
  int cam_height_ = -1;
  float cam_wfov_deg_ = -1.0;

  float focal_ = -1.0;
};

}  // namespace unrealcv_bridge
