#include "act_map/interpolation.h"

#include <random>

#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>

#include "act_map/common.h"
#include "act_map/voxblox_utils.h"

using namespace act_map;

class InterpolationTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::string dir;
    std::string fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    std::string abs_vis_cfg_folder = dir + "/test_data/fov50_fs50_lm1000_k10";

    GPVisApproxPtr gp_vis_ptr = std::make_shared<GPVisibilityApproximator>();
    gp_vis_ptr->load(abs_vis_cfg_folder);
    GPTraceVoxel::setVisApprox(gp_vis_ptr);

    LayerOptions options;
    options.vox_size = 0.5;
    options.vox_per_side = 4;
    std::cout << "factor voxel size: " << options.vox_size << "; "
              << "# voxesl per side: " << options.vox_per_side << std::endl;
    layer_ptr_.reset(new GPTraceLayer(options.vox_size, options.vox_per_side));
    layer_ptr_->allocateBlockPtrByCoordinates(blk0_pt_);

    voxblox::BlockIndexList blk_idxs;
    layer_ptr_->getAllAllocatedBlocks(&blk_idxs);
    blk0_ptr_ = layer_ptr_->getBlockPtrByIndex(blk_idxs[0]);
    EXPECT_TRUE(blk0_ptr_.get() != nullptr);
  }

  GPTraceLayer::Ptr layer_ptr_;
  GPTraceBlock::Ptr blk0_ptr_;
  Eigen::Vector3d blk0_pt_ = Eigen::Vector3d(0.9, 0.9, 0.9);
};

TEST_F(InterpolationTest, testFindVoxelsSpecialCases)
{
  // far
  Eigen::Vector3d far_pt(100.0, 100.0, 100.0);
  InterpVoxels<GPTraceVoxel> out{ nullptr };
  InterpVoxCenters centers;
  centers.setZero();
  QueryVoxelsRes res = getSurroundVoxels(*layer_ptr_, far_pt, &out, &centers);
  EXPECT_EQ(QueryVoxelsRes::kPointOutside, res);

  // get some block
  voxblox::BlockIndexList blk_idxs;
  layer_ptr_->getAllAllocatedBlocks(&blk_idxs);

  voxblox::VoxelIndex vox_idx;
  // query at the surface
  std::cout << "> Query on the surface:\n";
  vox_idx.x() = 0;
  vox_idx.y() = blk0_ptr_->maxIdx() / 2;
  vox_idx.z() = blk0_ptr_->maxIdx() / 2;
  EXPECT_TRUE(blk0_ptr_->isValidVoxelIndex(vox_idx));
  Eigen::Vector3d c = blk0_ptr_->computeCoordinatesFromVoxelIndex(vox_idx);
  std::cout << "Query at point: " << c.transpose() << "with index "
            << vox_idx.transpose() << std::endl;
  res = getSurroundVoxels(*layer_ptr_, c, &out, &centers);
  EXPECT_EQ(QueryVoxelsRes::kNearest, res);
  c = c - Eigen::Vector3d(0.1, 0, 0);
  std::cout << "Query at point: " << c.transpose() << "with index "
            << vox_idx.transpose() << std::endl;
  res = getSurroundVoxels(*layer_ptr_, c, &out, &centers);
  EXPECT_EQ(QueryVoxelsRes::kNoAdjacentBlocks, res);

  // query at the corner
  std::cout << "> Query at the corner:\n";
  vox_idx.x() = blk0_ptr_->maxIdx();
  vox_idx.y() = blk0_ptr_->maxIdx();
  vox_idx.z() = blk0_ptr_->maxIdx();
  c = blk0_ptr_->computeCoordinatesFromVoxelIndex(vox_idx);
  std::cout << "Query at point: " << c.transpose() << "with index "
            << vox_idx.transpose() << std::endl;
  res = getSurroundVoxels(*layer_ptr_, c, &out, &centers);
  EXPECT_EQ(QueryVoxelsRes::kNearest, res);
  c(0) += 0.1;
  std::cout << "Query at point: " << c.transpose() << "with index "
            << vox_idx.transpose() << std::endl;
  res = getSurroundVoxels(*layer_ptr_, c, &out, &centers);
  EXPECT_EQ(QueryVoxelsRes::kNoAdjacentBlocks, res);

  // query at the edge
  std::cout << "> Query on the edge:\n";
  vox_idx.x() = 1;
  vox_idx.y() = 0;
  vox_idx.z() = 0;
  c = blk0_ptr_->computeCoordinatesFromVoxelIndex(vox_idx);
  std::cout << "Query at point: " << c.transpose() << "with index "
            << vox_idx.transpose() << std::endl;
  res = getSurroundVoxels(*layer_ptr_, c, &out, &centers);
  EXPECT_EQ(QueryVoxelsRes::kNearest, res);
  c(1) -= 0.1;
  std::cout << "Query at point: " << c.transpose() << " with index "
            << vox_idx.transpose() << std::endl;
  res = getSurroundVoxels(*layer_ptr_, c, &out, &centers);
  EXPECT_EQ(QueryVoxelsRes::kNoAdjacentBlocks, res);
}

TEST_F(InterpolationTest, testFindVoxelsSuccess)
{
  // allocate another block
  Eigen::Vector3d blk1_pt = blk0_pt_;
  blk1_pt(0) += layer_ptr_->block_size();
  layer_ptr_->allocateNewBlockByCoordinates(blk1_pt);
  voxblox::BlockIndexList blk_idxs;
  layer_ptr_->getAllAllocatedBlocks(&blk_idxs);
  std::cout << "Allocated " << blk_idxs.size() << " blocks.\n";
  for (const voxblox::BlockIndex& blk_idx : blk_idxs)
  {
    GPTraceBlock::Ptr blk_ptr = layer_ptr_->getBlockPtrByIndex(blk_idx);
    std::cout << " - at " << blk_ptr->center().transpose() << " with size "
              << blk_ptr->block_size() << std::endl;
  }
  GPTraceBlock::Ptr blk1_ptr = layer_ptr_->getBlockPtrByCoordinates(blk1_pt);
  EXPECT_TRUE(blk1_ptr.get() != nullptr);

  // in-block query
  Eigen::Vector3d c0 = blk0_ptr_->center();
  InterpVoxels<GPTraceVoxel> out{ nullptr };
  InterpVoxCenters centers;
  QueryVoxelsRes res = getSurroundVoxels(*layer_ptr_, c0, &out, &centers);
  EXPECT_EQ(QueryVoxelsRes::kSurround, res);
  for (const auto& v : out)
  {
    EXPECT_TRUE(v != nullptr);
  }
  std::cout << "Queried voxels are:\n";
  for (int i = 0; i < 8; i++)
  {
    std::cout << " - " << centers.col(i).transpose() << std::endl;
  }
  std::cout << "......surrouding " << c0.transpose() << std::endl;

  // cross-block query
  voxblox::VoxelIndex x_max_idx;
  x_max_idx << blk0_ptr_->maxIdx(), 1, 1;
  Eigen::Vector3d x_max_c =
      blk0_ptr_->computeCoordinatesFromVoxelIndex(x_max_idx);
  x_max_c(0) += 0.1;
  res = getSurroundVoxels(*layer_ptr_, x_max_c, &out, &centers);
  EXPECT_EQ(QueryVoxelsRes::kSurround, res);
  for (const auto& v : out)
  {
    EXPECT_TRUE(v != nullptr);
  }
  std::cout << "Queried voxels are:\n";
  for (int i = 0; i < 8; i++)
  {
    std::cout << " - " << centers.col(i).transpose() << std::endl;
  }
  std::cout << "......surrouding " << x_max_c.transpose() << std::endl;

  // not the other direction
  voxblox::VoxelIndex y_max_idx;
  y_max_idx << 1, blk0_ptr_->maxIdx(), 1;
  Eigen::Vector3d y_max_c =
      blk0_ptr_->computeCoordinatesFromVoxelIndex(y_max_idx);
  y_max_c(1) += 0.1;
  res = getSurroundVoxels(*layer_ptr_, y_max_c, &out, &centers);
  EXPECT_EQ(QueryVoxelsRes::kNoAdjacentBlocks, res);
}

TEST_F(InterpolationTest, testInterpolationValue)
{
  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_int_distribution<> dis_linear_idx(
      0, static_cast<int>(blk0_ptr_->num_voxels() - 1));
  std::uniform_int_distribution<> dis_inside(1, blk0_ptr_->maxIdx() - 1);

  // set values to be increasing with linear index
  for (size_t i = 0; i < blk0_ptr_->num_voxels(); i++)
  {
    GPTraceVoxel& vox_i = blk0_ptr_->getVoxelByLinearIndex(i);
    vox_i.getFactorRef().setConstant(i);
  }

  // random voxel within the block
  voxblox::VoxelIndex rvox_idx;
  rvox_idx.x() = dis_inside(gen);
  rvox_idx.y() = dis_inside(gen);
  rvox_idx.z() = dis_inside(gen);
  size_t rlin_idx = blk0_ptr_->computeLinearIndexFromVoxelIndex(rvox_idx);
  Eigen::Vector3d q_pt = blk0_ptr_->computeCoordinatesFromVoxelIndex(rvox_idx);
  InterpVoxels<GPTraceVoxel> out_vox{ nullptr };
  InterpVoxCenters out_vox_c;
  QueryVoxelsRes res =
      getSurroundVoxels(*layer_ptr_, q_pt, &out_vox, &out_vox_c);
  EXPECT_EQ(QueryVoxelsRes::kNearest, res);
  EXPECT_DOUBLE_EQ(out_vox[0]->getFactorConstRef()(0, 0), rlin_idx);
  for (size_t i = 1; i < out_vox.size(); i++)
  {
    EXPECT_TRUE(out_vox[i] == nullptr);
  }

  //
  std::array<double, 5> test_ratios{ 0.1, 0.5, 0.9, -0.1, -0.9 };
  for (double r : test_ratios)
  {
    Eigen::Vector3d off_pt = q_pt.array() + r * blk0_ptr_->voxel_size();
    std::cout << ">> Test for: r " << r << ", point " << off_pt.transpose()
              << std::endl;
    res = getSurroundVoxels(*layer_ptr_, off_pt, &out_vox, &out_vox_c);
    EXPECT_EQ(QueryVoxelsRes::kSurround, res);
    std::cout << " Get voxles:\n";
    InterpValues<GPTraceVoxel::FactorType> vox_values;
    for (size_t i = 0; i < out_vox.size(); i++)
    {
      EXPECT_TRUE(out_vox[i] != nullptr);
      std::cout << " - " << out_vox_c.col(i).transpose() << " with value "
                << out_vox[i]->getFactorConstRef()(0, 0) << std::endl;
      vox_values[i] = &out_vox[i]->getFactorConstRef();
    }
    // trilinear interpolation
    GPTraceVoxel::FactorType interp_v;
    trilinearInterpolation(off_pt, out_vox_c.col(0), blk0_ptr_->voxel_size(),
                           vox_values, &interp_v);
    std::cout << " interpolated value: " << interp_v.transpose() << std::endl;
    EXPECT_GT(interp_v(0, 0), (*vox_values.front())(0, 0));
    EXPECT_LT(interp_v(0, 0), (*vox_values.back())(0, 0));
  }
}

RPG_COMMON_TEST_MAIN
{
}
