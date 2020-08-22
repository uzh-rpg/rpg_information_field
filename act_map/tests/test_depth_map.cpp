#include "act_map/depth_map.h"

#include <random>

#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>

using namespace act_map;

class DepthMapTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::string dir, fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    trace_dir_ = dir + "/../trace/tests";
    CHECK(rpg::fs::pathExists(trace_dir_)) << trace_dir_;
  }
  DepthMapOptions def_opts_;
  std::string trace_dir_;
};

TEST_F(DepthMapTest, init)
{
  DepthMap dm(def_opts_);
  std::cout << dm;

  CHECK(dm.depthLayerPtr());
  const DepthLayer& dl_ref = dm.depthLayerCRef();
  EXPECT_EQ(0u, dl_ref.getNumberOfAllocatedBlocks());
}

TEST_F(DepthMapTest, allocationPerPoint)
{
  DepthMap dm(def_opts_);

  Eigen::Vector3d rpt;
  rpt.setRandom();

  dm.allocateByPoints({ rpt });
  EXPECT_GT(dm.numAllocatedBlocks(), 0u);
  EXPECT_GT(dm.numAllocatedVoxels(), 0u);

  EXPECT_EQ(dm.numInitializedVoxels(), dm.numAllocatedVoxels());

  EXPECT_EQ(dm.rawBlockIdx().size(), dm.numAllocatedBlocks());

  dm.reset();
  EXPECT_EQ(0, dm.numAllocatedBlocks());
  EXPECT_EQ(0, dm.numAllocatedVoxels());
  EXPECT_EQ(0, dm.rawBlockIdx().size());
}

TEST_F(DepthMapTest, allocationPerRange)
{
  DepthMap dm(def_opts_);
  EXPECT_EQ(0, dm.numAllocatedBlocks());
  dm.allocateUniformWithin({ 10.0, 10.0, 10.0 });
  EXPECT_GT(dm.numAllocatedBlocks(), 0);
  std::cout << "After allocation: " << dm.numAllocatedVoxels() << " voxels.\n";
  dm.reset();
  EXPECT_EQ(0, dm.numAllocatedVoxels());
}

TEST_F(DepthMapTest, getSetRawBlocks)
{
  DepthMap dm(def_opts_);
  DepthBlock::Ptr null_blk_ptr = dm.getDepthBlockPtr(voxblox::BlockIndex(10));
  EXPECT_FALSE(null_blk_ptr);

  Eigen::Vector3d rpt;
  rpt.setRandom();
  dm.allocateByPoints({ rpt });
  for (const voxblox::BlockIndex bidx : dm.rawBlockIdx())
  {
    DepthBlock::Ptr blk_ptr = dm.getDepthBlockPtr(bidx);
    EXPECT_TRUE(blk_ptr);
  }
  EXPECT_EQ(1u, dm.rawBlockIdx().size());

  while (!dm.rawBlockIdx().empty())
  {
    voxblox::BlockIndex bidx = dm.consumeRawBlockIdx();
    DepthBlock::Ptr blk_ptr = dm.getDepthBlockPtr(bidx);
    EXPECT_TRUE(blk_ptr);
  }
  EXPECT_EQ(0u, dm.rawBlockIdx().size());
}

TEST_F(DepthMapTest, saveAndLoad)
{
  DepthMapOptions options;
  options.depth_layer_opts_.vox_size = 0.5;
  options.depth_layer_opts_.vox_per_side = 4;
  options.depth_voxel_step_deg_ = 2.0;
  std::vector<double> ranges{ 5.0, 5.0, 3.0 };
  DepthMap dm(options);
  dm.allocateUniformWithin(ranges);
  std::cout << "Number of allocated voxels: " << dm.numAllocatedVoxels()
            << std::endl;

  while (!dm.rawBlockIdx().empty())
  {
    voxblox::BlockIndex bidx = dm.consumeRawBlockIdx();
    dm.setBlockConstant(bidx, 10.0);
  }
  const std::string fn = trace_dir_ + "/depth_map.proto";
  dm.saveDepthLayer(fn);
  std::cout << "Saved layer.\n";

  DepthMap dml(options);
  dml.loadDepthLayer(fn);

  EXPECT_EQ(dm.numAllocatedBlocks(), dml.numAllocatedBlocks());
  EXPECT_EQ(dm.numAllocatedVoxels(), dml.numAllocatedVoxels());
  EXPECT_EQ(dm.numInitializedVoxels(), dml.numInitializedVoxels());
  EXPECT_GT(dm.numInitializedVoxels(), 0);
  const int n_test = 100;
  rpg::PositionVec rd_points;
  utils::generateRandomPointsWithin(n_test, -ranges[0] / 2, ranges[0] / 2,
                                    -ranges[1] / 2, ranges[1] / 2,
                                    -ranges[2] / 2, ranges[2] / 2, &rd_points);

  for (const Eigen::Vector3d& pt : rd_points)
  {
    DepthBlock::ConstPtr org_blk =
        dm.depthLayerPtr()->getBlockPtrByCoordinates(pt);
    DepthBlock::ConstPtr load_blk =
        dml.depthLayerPtr()->getBlockPtrByCoordinates(pt);
    EXPECT_EQ(org_blk->num_voxels(), load_blk->num_voxels());
    for (size_t i = 0; i < org_blk->num_voxels(); i++)
    {
      const DepthVoxel& org_vox = org_blk->getVoxelByLinearIndex(i);
      const DepthVoxel& load_vox = load_blk->getVoxelByLinearIndex(i);
      EXPECT_TRUE(
          org_vox.rayDepthMatCRef().isApprox(load_vox.rayDepthMatCRef()));
    }
  }
}

TEST_F(DepthMapTest, visibility)
{
  DepthMapOptions options;
  options.depth_layer_opts_.vox_size = 0.5;
  options.depth_layer_opts_.vox_per_side = 4;
  options.depth_voxel_step_deg_ = 2.0;
  DepthMap dm(options);
  dm.allocateByPoints({ Eigen::Vector3d::Zero() });
  std::cout << "Number of allocated voxels: " << dm.numAllocatedVoxels()
            << std::endl;
  std::cout << "Number of allocated blocks: " << dm.numAllocatedBlocks()
            << std::endl;

  std::random_device rd;
  std::mt19937 gen(rd());
  rpg::PositionVec points_query;
  points_query.push_back(Eigen::Vector3d(3.0, 3.0, 3.0));
  points_query.push_back(Eigen::Vector3d(20.0, 20.0, 20.0));
  std::vector<VisStatus> vis;

  std::uniform_real_distribution<double> dis_neg(-2.0, -0.1);
  std::uniform_real_distribution<double> dis_pos(0.1, 2.0);
  for (size_t i = 0; i < 10; i++)
  {
    Eigen::Vector3d rc_neg(dis_neg(gen), dis_neg(gen), dis_neg(gen));
    dm.queryPointsVisibilityAt(rc_neg, points_query, &vis);
    for (const auto v : vis)
    {
      EXPECT_EQ(v, VisStatus::kNotCovered);
    }
    Eigen::Vector3d rc_pos(dis_pos(gen), dis_pos(gen), dis_pos(gen));
    dm.queryPointsVisibilityAt(rc_pos, points_query, &vis);
    for (const auto v : vis)
    {
      EXPECT_EQ(v, VisStatus::kUnknownDepth);
    }
  }

  while (!dm.rawBlockIdx().empty())
  {
    voxblox::BlockIndex bidx = dm.consumeRawBlockIdx();
    dm.setBlockConstant(bidx, 5.0);
  }

  for (size_t i = 0; i < 10; i++)
  {
    Eigen::Vector3d rc(dis_pos(gen), dis_pos(gen), dis_pos(gen));
    dm.queryPointsVisibilityAt(rc, points_query, &vis);
    EXPECT_EQ(vis[0], VisStatus::kVisibile);
    EXPECT_EQ(vis[1], VisStatus::kOccluded);
  }
}

RPG_COMMON_TEST_MAIN
{
}
