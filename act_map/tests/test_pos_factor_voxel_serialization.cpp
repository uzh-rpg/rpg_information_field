#include "act_map/positional_factor_voxel_ops.h"

#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>

#include "act_map/conversion.h"

using namespace act_map;

template <typename T>
class PosFactorVoxelSerializationTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::string dir;
    std::string fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    std::string vis_dir = dir + "/test_data/fov45_fs30_lm1000_k10_fast";

    setGPVisiblityFromFolder(vis_dir);
    setQuadPolyVisiblity(QuadVisScoreOptions());

    points_.resize(Eigen::NoChange, 100);
    points_.setRandom();

    org_.setRandom();
  }

  Eigen::Vector3d org_;
  Eigen::Matrix3Xd points_;
};

using PosFactorVoxelTypes =
    ::testing::Types<QuadInfoVoxel, QuadTraceVoxel, GPInfoVoxel, GPTraceVoxel,
                     QuadPolyInfoVoxel, QuadPolyTraceVoxel>;
TYPED_TEST_CASE(PosFactorVoxelSerializationTest, PosFactorVoxelTypes);

TYPED_TEST(PosFactorVoxelSerializationTest, testSize)
{
  using VoxelT = TypeParam;
  VoxelT vox;
  std::cout << "Serialization size:\n";
  std::cout << "- " << getVoxelType<VoxelT>() << ": " << vox.serializationSize()
            << std::endl;
}

TYPED_TEST(PosFactorVoxelSerializationTest, serializationAndDeserialization)
{
  using VoxelT = TypeParam;
  VoxelT vox;
  addToFactorVoxel(this->points_, this->org_, &vox);

  {
    std::vector<uint64_t> buf64;
    vox.serializeToIntegers(&buf64);
    EXPECT_EQ(vox.serializationSize(), buf64.size());
    VoxelT d_vox_64;
    d_vox_64.deserializeFromIntegers(buf64, 0);
    EXPECT_TRUE(d_vox_64.isTheSame(vox, 1e-8));
  }

  {
    std::vector<uint32_t> buf32;
    vox.serializeToIntegers(&buf32);
    EXPECT_EQ(vox.serializationSize(), buf32.size());
    VoxelT d_vox_32;
    d_vox_32.deserializeFromIntegers(buf32, 0);
    EXPECT_TRUE(d_vox_32.isTheSame(vox, 1e-3));
  }
}

RPG_COMMON_TEST_MAIN
{
}
