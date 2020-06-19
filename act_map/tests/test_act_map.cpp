//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map/act_map.h"

#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>

#include <vi_utils/map.h>
#include <vi_utils/states.h>
#include <vi_utils/cam_min.h>

using namespace act_map;

class ActMapTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::string dir;
    std::string fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    abs_map_ = dir + "/test_data/test_map.txt";
    abs_traj_ = dir + "/test_data/test_traj.csv";

    map_.reset(new vi::Map());
    map_->load(abs_map_, std::string());
    vi::States::load(abs_traj_, &states_, ',', true);
  }

  std::string abs_map_;
  std::string abs_traj_;

  vi::MapPtr map_;
  vi::StatesVec states_;

  double xrange_ = 4.0;
  double yrange_ = 4.0;
  double zrange_ = 2.0;
};

TEST_F(ActMapTest, testInitPrint)
{
  ActMapOptions options;
  options.vis_options_.resize(1);
  TraceMap am(options);

  std::cout << am;
}

TEST_F(ActMapTest, testOccupancyIntegrate)
{
  ActMapOptions options;
  options.occ_layer_options_.vox_size = 0.01;
  options.vis_options_.resize(1);
  InfoMap am(options);

  EXPECT_EQ(0u, am.occLayerCRef().getNumberOfAllocatedBlocks());
  std::cout << "Occ Memory at the beginning: "
            << am.occLayerCRef().getMemorySize() << std::endl;

  rpg::Pose cam_pose;
  cam_pose.setIdentity();  // at origin
  am.integratePointCloudOccupancy(cam_pose, map_->points_);

  EXPECT_EQ(0, am.getLastAddedOccPoints().size());
  EXPECT_EQ(0, am.getLastDeletedOccPoints().size());

  for (int i = 0; i < 10; i++)
  {
    am.integratePointCloudOccupancy(cam_pose, map_->points_);
    if (am.getLastAddedOccPoints().size() > 0)
    {
      break;
    }
  }
  size_t n_pts = 0;
  for (const auto& pair : am.getLastAddedOccPoints())
  {
    n_pts += pair.second.size();
  }
  EXPECT_EQ(map_->points_.cols(), n_pts);
  EXPECT_EQ(0, am.getLastDeletedOccPoints().size());

  EXPECT_GT(am.occLayerCRef().getNumberOfAllocatedBlocks(), 0u);
  std::cout << "Occ Memory after integration (KB): "
            << am.occLayerCRef().getMemorySize() << std::endl;
  std::cout << "Number of occ blocks after integration: "
            << am.occLayerCRef().getNumberOfAllocatedBlocks() << std::endl;
}

TEST_F(ActMapTest, testAllocationKernelLayer)
{
  ActMapOptions options;
  options.vis_options_.resize(1);
  TraceMap am(options);

  EXPECT_EQ(0, am.kerLayerCRef().getNumberOfAllocatedBlocks());

  std::vector<double> ranges{ -xrange_ / 2, xrange_ / 2,  -yrange_ / 2,
                              yrange_ / 2,  -zrange_ / 2, zrange_ / 2 };
  am.allocateKernelLayerUniform(ranges);
  EXPECT_GT(am.kerLayerCRef().getNumberOfAllocatedBlocks(), 0);

  am.kerLayerPtr()->removeAllBlocks();
  EXPECT_EQ(0, am.kerLayerCRef().getNumberOfAllocatedBlocks());
  std::vector<double> ranges2{ xrange_, yrange_, zrange_ };
  am.allocateKernelLayerUniform(ranges2);
  EXPECT_GT(am.kerLayerCRef().getNumberOfAllocatedBlocks(), 0);
}

TEST_F(ActMapTest, testKernelIntegrationBatch)
{
  ActMapOptions options;
  options.vis_options_.resize(1);
  TraceMap am(options);

  // set occupancy
  EXPECT_DOUBLE_EQ(0.0, am.updatedBlkRatioOcc());
  rpg::Pose cam_pose;
  cam_pose.setIdentity();  // at origin
  for (size_t i = 0; i < 10; i++)
  {
    am.integratePointCloudOccupancy(cam_pose, map_->points_);
  }
  EXPECT_DOUBLE_EQ(1.0, am.updatedBlkRatioOcc());

  // allocate
  EXPECT_DOUBLE_EQ(0.0, am.accumulatedUpdatedBlkRatioKernel());
  std::vector<double> ranges{ xrange_, yrange_, zrange_ };
  am.allocateKernelLayerUniform(ranges);

  EXPECT_DOUBLE_EQ(0.0, am.updatedBlkRatioKernel());
  EXPECT_DOUBLE_EQ(0.0, am.accumulatedUpdatedBlkRatioKernel());
  voxblox::BlockIndexList updated_blk_idxs;
  const TraceLayer& tl = am.kerLayerCRef();
  tl.getAllUpdatedBlocks(&updated_blk_idxs);
  EXPECT_TRUE(updated_blk_idxs.empty());
  am.recomputeKernelLayer();
  tl.getAllUpdatedBlocks(&updated_blk_idxs);
  EXPECT_FALSE(updated_blk_idxs.empty());
  EXPECT_EQ(tl.getNumberOfAllocatedBlocks(), updated_blk_idxs.size());
  EXPECT_DOUBLE_EQ(1.0, am.updatedBlkRatioKernel());
  EXPECT_DOUBLE_EQ(1.0, am.accumulatedUpdatedBlkRatioKernel());

  const voxblox::BlockIndexList& last_updated_kblk_idxs =
      am.getAccumulatedUpdatedKernelBlocksIndices();
  EXPECT_EQ(utils::getNumOfUpdatedBlocks(tl), last_updated_kblk_idxs.size());
}

TEST_F(ActMapTest, testKernelUpdateIncremental)
{
  ActMapOptions options;
  options.vis_options_.resize(1);
  TraceMap am_batch(options);
  TraceMap am_inc(options);
  std::vector<double> ranges{ xrange_, yrange_, zrange_ };
  am_batch.allocateKernelLayerUniform(ranges);
  am_inc.allocateKernelLayerUniform(ranges);
  EXPECT_TRUE(voxblox::utils::isSameLayer(am_batch.kerLayerCRef(),
                                          am_inc.kerLayerCRef()));

  // batch
  utils::setPointsInOccupancyLayer(map_->points_, am_batch.occLayerPtr().get());
  am_batch.recomputeKernelLayer();

  // incremental
  rpg::Pose cam_pose;
  cam_pose.setIdentity();  // at origin
  for (size_t i = 0; i < 10; i++)
  {
    am_inc.integratePointCloudOccupancy(cam_pose, map_->points_);
    am_inc.updateKernelLayerIncremental();
  }

  //
  EXPECT_TRUE(voxblox::utils::isSameLayer(am_batch.occLayerCRef(),
                                          am_inc.occLayerCRef()));
  EXPECT_TRUE(voxblox::utils::isSameLayer(am_batch.kerLayerCRef(),
                                          am_inc.kerLayerCRef()));
}

TEST_F(ActMapTest, testAddLocationInterface)
{
  ActMapOptions options;
  options.vis_options_.resize(1);
  TraceMap am(options);
  utils::setPointsInOccupancyLayer(map_->points_, am.occLayerPtr().get());

  rpg::Pose Twb;
  Twb.setIdentity();
  const std::vector<double> ranges{ 1, 1, 1 };
  am.addRegionToKernelLayer(Twb, ranges);
  EXPECT_GT(am.getNewlyAllocatedKernelBlockIndices().size(), 0);
  am.updateKernelLayerIncremental();
  EXPECT_EQ(am.getNewlyAllocatedKernelBlockIndices().size(), 0);
}

RPG_COMMON_TEST_MAIN
{
}
