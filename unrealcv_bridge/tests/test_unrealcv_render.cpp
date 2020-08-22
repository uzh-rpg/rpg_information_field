#include "unrealcv_bridge/unrealcv_render.h"

#include <sstream>

#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>

#include <opencv2/imgcodecs.hpp>

using namespace unrealcv_bridge;

class UnrealCVRenderTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::string cpath, cfn;
    rpg_common::fs::splitPathAndFilename(__FILE__, &cpath, &cfn);
    ue_pose_fn_ =
        rpg_common::fs::concatenateFolderAndFileName(
            cpath, "../test_data/poses_xyzpyr_ue_warehouse.txt");
    save_dir_ =
        rpg_common::fs::concatenateFolderAndFileName(
            cpath, "../trace/test_unrealcv_render");
    CHECK(rpg_common::fs::fileExists(ue_pose_fn_));
    CHECK(rpg_common::fs::pathExists(save_dir_));
  }

  std::string ue_pose_fn_;
  std::string save_dir_;
};

TEST_F(UnrealCVRenderTest, testInitialize)
{
  UnrealCVRender render;
  render.disconnect();
}

TEST_F(UnrealCVRenderTest, testLoadTraj)
{
  UnrealCVRender render;
  render.loadUEPoses(ue_pose_fn_);

  EXPECT_EQ(389u, render.numPoses());
}

TEST_F(UnrealCVRenderTest, renderUEPose)
{
  UnrealCVRender render;
  render.loadUEPoses(ue_pose_fn_);
  render.focal() = 320.0f;

  cv::Mat img;
  cv::Mat ray_depth;
  cv::Mat z_depth;
  constexpr size_t n_img = 5;
  for (size_t i = 0; i < n_img; i++)
  {
    std::ostringstream ss;
    ss << std::setfill('0') << std::setw(5) << i;
    std::cout << "Saving " << ss.str() << "..." << std::endl;
    UEPose uep = render.getUEPoses(i*10);

    render.renderImg(uep, &img);
    cv::imwrite(save_dir_ + "/" + ss.str() + ".png", img);

    render.renderDepth(uep, DepthMode::kRayDepth, &ray_depth);
    cv::imwrite(save_dir_ + "/depth_ray_" + ss.str() + ".png", ray_depth);

    render.renderDepth(uep, DepthMode::kZDepth, &z_depth);
    cv::imwrite(save_dir_ + "/depth_z_" + ss.str() + ".png", z_depth);

    EXPECT_GT(img.rows, 0);
    EXPECT_GT(img.cols, 0);
    render.sleep(500);
  }
}

TEST_F(UnrealCVRenderTest, renderTraj)
{

}


RPG_COMMON_TEST_MAIN
{

}
