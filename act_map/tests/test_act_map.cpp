#include "act_map/act_map.h"

#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>

#include <vi_utils/map.h>
#include <vi_utils/states.h>
#include <vi_utils/cam_min.h>

using namespace act_map;

template <typename T>
class ActMapTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::string dir;
    std::string fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    abs_map_ = dir + "/test_data/test_map_100.txt";
    abs_traj_ = dir + "/test_data/test_traj.csv";

    std::string vis_dir = dir + "/test_data/fov45_fs30_lm1000_k10_fast";
    setGPVisiblityFromFolder(vis_dir);

    map_.reset(new vi::Map());
    map_->load(abs_map_, std::string());
    vi::States::load(abs_traj_, &states_, ',', true);

    ranges_ = std::vector<double>{ -this->xrange_ / 2, this->xrange_ / 2,
                                   -this->yrange_ / 2, this->yrange_ / 2,
                                   -this->zrange_ / 2, this->zrange_ / 2 };

    options_.pos_factor_layer_options_.vox_size = 0.3;
    options_.occ_layer_options_.vox_size = 0.01;
    options_.vis_options_.resize(1);

    setQuadPolyVisiblity(options_.vis_options_[0]);
  }

  std::string abs_map_;
  std::string abs_traj_;

  vi::MapPtr map_;
  vi::StatesVec states_;

  double xrange_ = 1.0;
  double yrange_ = 1.0;
  double zrange_ = 1.0;

  std::vector<double> ranges_;

  ActMapOptions options_;
};

using PosFactorVoxelTypes =
    ::testing::Types<QuadInfoVoxel, QuadTraceVoxel, GPInfoVoxel, GPTraceVoxel,
                     QuadPolyInfoVoxel, QuadPolyTraceVoxel>;
TYPED_TEST_CASE(ActMapTest, PosFactorVoxelTypes);

TYPED_TEST(ActMapTest, testInitPrint)
{
  ActMap<TypeParam> am(this->options_);

  std::cout << am;
}

TYPED_TEST(ActMapTest, testOccupancyIntegrate)
{
  ActMap<TypeParam> am(this->options_);

  EXPECT_EQ(0u, am.occLayerCRef().getNumberOfAllocatedBlocks());
  std::cout << "Occ Memory at the beginning: "
            << am.occLayerCRef().getMemorySize() << std::endl;

  rpg::Pose cam_pose;
  cam_pose.setIdentity();  // at origin
  am.integratePointCloudOccupancy(cam_pose, this->map_->points_);

  EXPECT_EQ(0, am.getLastAddedOccPoints().size());
  EXPECT_EQ(0, am.getLastDeletedOccPoints().size());

  for (int i = 0; i < 10; i++)
  {
    am.integratePointCloudOccupancy(cam_pose, this->map_->points_);
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
  EXPECT_EQ(this->map_->points_.cols(), n_pts);
  EXPECT_EQ(0, am.getLastDeletedOccPoints().size());

  EXPECT_GT(am.occLayerCRef().getNumberOfAllocatedBlocks(), 0u);
  std::cout << "Occ Memory after integration (KB): "
            << am.occLayerCRef().getMemorySize() << std::endl;
  std::cout << "Number of occ blocks after integration: "
            << am.occLayerCRef().getNumberOfAllocatedBlocks() << std::endl;
}

TYPED_TEST(ActMapTest, testAllocationKernelLayer)
{
  ActMap<TypeParam> am(this->options_);

  EXPECT_EQ(0, am.kerLayerCRef().getNumberOfAllocatedBlocks());
  am.allocateFactorLayerUniform(this->ranges_);
  EXPECT_GT(am.kerLayerCRef().getNumberOfAllocatedBlocks(), 0);

  am.kerLayerPtr()->removeAllBlocks();
  EXPECT_EQ(0, am.kerLayerCRef().getNumberOfAllocatedBlocks());
  std::vector<double> ranges2{ this->xrange_, this->yrange_, this->zrange_ };
  am.allocateFactorLayerUniform(ranges2);
  EXPECT_GT(am.kerLayerCRef().getNumberOfAllocatedBlocks(), 0);
}

TYPED_TEST(ActMapTest, testKernelIntegrationBatch)
{
  ActMap<TypeParam> am(this->options_);

  // set occupancy
  EXPECT_EQ(0, am.numOccupiedVoxels());
  am.setOccupancyWorldPoints(this->map_->points_);
  EXPECT_EQ(this->map_->n_points_, am.numOccupiedVoxels());

  // allocate
  EXPECT_DOUBLE_EQ(0.0, am.accumulatedUpdatedBlkRatioFactorLayer());
  std::vector<double> ranges{ this->xrange_, this->yrange_, this->zrange_ };
  am.allocateFactorLayerUniform(ranges);

  EXPECT_DOUBLE_EQ(0.0, am.updatedBlkRatioFactorLayer());
  EXPECT_DOUBLE_EQ(0.0, am.accumulatedUpdatedBlkRatioFactorLayer());
  voxblox::BlockIndexList updated_blk_idxs;
  const voxblox::Layer<TypeParam>& tl = am.kerLayerCRef();
  tl.getAllUpdatedBlocks(&updated_blk_idxs);
  EXPECT_TRUE(updated_blk_idxs.empty());
  am.recomputeFactorLayer();
  tl.getAllUpdatedBlocks(&updated_blk_idxs);
  EXPECT_FALSE(updated_blk_idxs.empty());
  EXPECT_EQ(tl.getNumberOfAllocatedBlocks(), updated_blk_idxs.size());
  EXPECT_DOUBLE_EQ(1.0, am.updatedBlkRatioFactorLayer());
  EXPECT_DOUBLE_EQ(1.0, am.accumulatedUpdatedBlkRatioFactorLayer());

  const voxblox::BlockIndexList& last_updated_kblk_idxs =
      am.getAccumulatedUpdatedFactorBlocksIndices();
  EXPECT_EQ(utils::getNumOfUpdatedBlocks(tl), last_updated_kblk_idxs.size());
}

TYPED_TEST(ActMapTest, testKernelUpdateIncremental)
{
  ActMap<TypeParam> am_batch(this->options_);
  ActMap<TypeParam> am_inc(this->options_);
  std::vector<double> ranges{ this->xrange_, this->yrange_, this->zrange_ };
  am_batch.allocateFactorLayerUniform(ranges);
  am_inc.allocateFactorLayerUniform(ranges);
  EXPECT_TRUE(voxblox::utils::isSameLayer(am_batch.kerLayerCRef(),
                                          am_inc.kerLayerCRef()));

  // batch
  utils::setPointsInOccupancyLayer(this->map_->points_,
                                   am_batch.occLayerPtr().get());
  am_batch.recomputeFactorLayer();

  // incremental
  rpg::Pose cam_pose;
  cam_pose.setIdentity();  // at origin
  for (size_t i = 0; i < 10; i++)
  {
    am_inc.integratePointCloudOccupancy(cam_pose, this->map_->points_);
    am_inc.updateFactorLayerIncremental();
  }

  //
  EXPECT_TRUE(voxblox::utils::isSameLayer(am_batch.occLayerCRef(),
                                          am_inc.occLayerCRef()));
  EXPECT_TRUE(voxblox::utils::isSameLayer(am_batch.kerLayerCRef(),
                                          am_inc.kerLayerCRef()));
}

TYPED_TEST(ActMapTest, testAddLocationInterface)
{
  ActMap<TypeParam> am(this->options_);
  utils::setPointsInOccupancyLayer(this->map_->points_, am.occLayerPtr().get());

  rpg::Pose Twb;
  Twb.setIdentity();
  const std::vector<double> ranges{ 1, 1, 1 };
  am.addRegionToFactorLayer(Twb, ranges);
  EXPECT_GT(am.getNewlyAllocatedFactorBlockIndices().size(), 0);
  am.updateFactorLayerIncremental();
  EXPECT_EQ(am.getNewlyAllocatedFactorBlockIndices().size(), 0);
}

TYPED_TEST(ActMapTest, testGetInfoMetric)
{
  ActMap<TypeParam> am(this->options_);
  Eigen::Matrix3Xd views;
  views.resize(Eigen::NoChange, 0);
  am.setOccupancyWorldPoints(this->map_->points_, views);
  am.allocateFactorLayerUniform(this->ranges_);
  am.recomputeFactorLayer();
  am.prepareInfoFromPointCloud();

  rpg::Pose Twc;
  Twc.setIdentity();
  InfoMetricType info_t = InfoMetricType::kTrace;

  double val_ker;
  Eigen::Vector3d dpos_ker, drot_g_ker;
  am.getInfoMetricAt(Twc, info_t, &val_ker, &dpos_ker, &drot_g_ker);
  double val_pc;
  Eigen::Vector3d dpos_pc, drot_g_pc;
  am.getInfoMetricFromPC(Twc, info_t, &val_pc, &dpos_pc, &drot_g_pc);
  std::cout << "Kernel vs. pc:" << std::endl;
  std::cout << "- " << val_ker << " <-> " << val_pc << std::endl;
  std::cout << "- " << dpos_ker.transpose() << " <-> " << dpos_pc.transpose()
            << std::endl;
  std::cout << "- " << drot_g_ker.transpose() << " <-> "
            << drot_g_pc.transpose() << std::endl;
}

RPG_COMMON_TEST_MAIN
{
}
