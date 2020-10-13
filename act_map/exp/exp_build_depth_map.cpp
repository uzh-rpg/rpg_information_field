#include "act_map/depth_map.h"

#include <chrono>
#include <thread>

#include <rpg_common/fs.h>
#include <rpg_common/load.h>
#include <rpg_common/main.h>

#include <unrealcv_bridge/unrealcv_render.h>
#include <unrealcv_bridge/ue_utils.hpp>

DEFINE_string(config_fn, "", "config file in cfg dir");
DEFINE_double(step_deg, 5.0, "sampling step for depth voxel");
DEFINE_bool(use_center_point, false, "use center point of the range");
DEFINE_double(sleep_sec, 0.0, "sleep for X sec after each depth");
DEFINE_bool(viz_depth, false, "stop and show the depth map");

using namespace act_map;
using namespace unrealcv_bridge;

struct ExpConfig
{
  double xmin;
  double xmax;
  double ymin;
  double ymax;
  double zmin;
  double zmax;
  double step_deg;
  double vox_size;
  size_t vox_per_side;
  void parse(const std::string& abs_fn)
  {
    Eigen::MatrixXd data;
    rpg::load(abs_fn, &data);
    CHECK_EQ(data.rows(), 1);
    CHECK_EQ(data.cols(), 8);
    xmin = data(0);
    xmax = data(1);
    ymin = data(2);
    ymax = data(3);
    zmin = data(4);
    zmax = data(5);
    vox_size = data(6);
    vox_per_side = static_cast<size_t>(data(7));
  }
  void print()
  {
    std::cout << "Experiment config:\n";
    std::cout << "- xmin: " << xmin << std::endl;
    std::cout << "- xmax: " << xmax << std::endl;
    std::cout << "- ymin: " << ymin << std::endl;
    std::cout << "- ymax: " << ymax << std::endl;
    std::cout << "- zmin: " << zmin << std::endl;
    std::cout << "- zmax: " << zmax << std::endl;
    std::cout << "- vox_size: " << vox_size << std::endl;
    std::cout << "- vox_per_side: " << vox_per_side << std::endl;
  }

  void ranges(std::vector<double>* ranges)
  {
    CHECK(ranges);
    (*ranges) = std::vector<double>{ xmin, xmax, ymin, ymax, zmin, zmax };
  }

  void center(Eigen::Vector3d* c)
  {
    (*c) = Eigen::Vector3d(0.5 * (xmin + xmax), 0.5 * (ymin + ymax),
                           0.5 * (zmin + zmax));
  }
};

RPG_COMMON_MAIN
{
  CHECK(!FLAGS_config_fn.empty());
  std::cout << "Build DepthMap with the connection to UnrealEigine.\n";

  std::cout << "Experiment parameters:\n"
            << "- config: " << FLAGS_config_fn << std::endl
            << "- step_deg: " << FLAGS_step_deg << std::endl
            << "- use_center_point: " << FLAGS_use_center_point << std::endl
            << "- wait_sec: " << FLAGS_sleep_sec << std::endl;

  const int sleep_ms = static_cast<int>(1000 * FLAGS_sleep_sec);

  std::string path;
  std::string fn;
  rpg::fs::splitPathAndFilename(__FILE__, &path, &fn);
  const std::string abs_cfg = path + "/../cfg/" + FLAGS_config_fn;
  const std::string trace_dir = path + "/../trace/build_depth_map";
  rpg::fs::pathExists(trace_dir);
  ExpConfig cfg;
  cfg.parse(abs_cfg);
  cfg.print();

  DepthMapOptions dm_opts;
  dm_opts.depth_layer_opts_.vox_size = cfg.vox_size;
  dm_opts.depth_layer_opts_.vox_per_side = cfg.vox_per_side;
  dm_opts.depth_voxel_step_deg_ = FLAGS_step_deg;
  DepthMap dm(dm_opts);
  if (FLAGS_use_center_point)
  {
    Eigen::Vector3d c;
    cfg.center(&c);
    dm.allocateByPoints({ c });
  }
  else
  {
    std::vector<double> ranges;
    cfg.ranges(&ranges);
    dm.allocateUniformWithin(ranges);
  }
  std::cout << "Allocated " << dm.numAllocatedBlocks() << " blocks "
            << "and " << dm.numAllocatedVoxels() << " voxels." << std::endl;

  UnrealCVRender render;
  rpg::Pose Tbc;
  Tbc.setIdentity();
  vi_utils::PinholeCamPtr cam_ptr = std::make_shared<vi_utils::PinholeCam>(
      std::vector<double>{ 320.0, 320.0, 319.5, 319.5, 640.0, 640.0 }, Tbc);
  render.focal() = static_cast<float>(cam_ptr->fx());
  std::cout << "The current camera parameters is :" << *cam_ptr << std::endl;
  std::cout << "NOTE: make sure to change the camera parameters in the configuration file of the "
            << "simulator BEFORE launching the simulation." << std::endl;

  // rendering
  voxblox::BlockIndexList all_blks;
  dm.depthLayerCRef().getAllAllocatedBlocks(&all_blks);
  size_t blk_cnt = 1;
  rpg::Timer timer;
  cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
  for (const voxblox::BlockIndex& bidx : all_blks)
  {
    timer.start();
    std::cout << "> Processing block " << blk_cnt++ << " in "
              << dm.numAllocatedBlocks() << std::endl;
    DepthBlock::Ptr blk_ptr = dm.getDepthBlockPtr(bidx);
    CHECK(blk_ptr);
    for (size_t vox_i = 0; vox_i < blk_ptr->num_voxels(); vox_i++)
    {
      DepthVoxel& vox = blk_ptr->getVoxelByLinearIndex(vox_i);

      rpg::Pose Twc;
      Twc.setIdentity();
      Twc.getRotation() = rpg::Rotation(Eigen::Vector3d(-M_PI / 2, 0, 0));
      Twc.getPosition() = vox.center();

      rpg::PoseVec Tws;
      DepthVoxel::generateSampleTws(Twc, false, &Tws);

      std::vector<cv::Mat> dimgs(Tws.size());
      vi_utils::PinholeCamVec cam_vec(Tws.size(), cam_ptr);

      float prev_sample = -1.0;
      for (size_t pose_i = 0; pose_i < Tws.size(); pose_i++)
      {
        VLOG(10) << "- pose #: " << pose_i << std::endl;
        UEPose uep;
        TwcToUEPose(Tws[pose_i], &uep);
        VLOG(10) << "  yaw/pitch/roll: " << uep.yaw << ", " << uep.pitch << ", "
                 << uep.roll << std::endl;
        size_t cnt = 0;
        while (true)
        {
          render.renderDepth(uep, DepthMode::kRayDepth, &(dimgs.at(pose_i)));
          float cur_sample = dimgs.at(pose_i).at<float>(320, 320);
          if (std::fabs(prev_sample - cur_sample) > 1e-4)
          {
            prev_sample = cur_sample;
            break;
          }
          else
          {
            std::cout << "renderer seems to not response for pose " << pose_i
                      << ", redo.\n";
          }
          cnt++;
          if (cnt > 5)
          {
            break;
          }
        }

        if (FLAGS_viz_depth)
        {
          cv::Mat viz_depth;
          cv::normalize(dimgs.at(pose_i), viz_depth, 0, 255, cv::NORM_MINMAX,
                        CV_8U);
          cv::imshow("depth", viz_depth);
          cv::waitKey();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
      }
      vox.setFromDepthImages(cam_vec, dimgs, Tws, false);
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
      VLOG(10) << "Maximum depth: " << vox.rayDepthMatRef().maxCoeff()
                << std::endl;
    }
    std::cout << "< Time consumed: " << timer.stop() << " for "
              << blk_ptr->num_voxels() << " voxels." << std::endl;
  }
  dm.saveDepthLayer(trace_dir + "/depthmap.proto");
  return 0;
}
