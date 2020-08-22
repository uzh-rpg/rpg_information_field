#include "act_map/depth_voxel.h"

#include <memory>
#include <random>

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>

using namespace act_map;

class DepthVoxelTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    org_.setZero();
    std::string dir, fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    trace_dir_ = dir + "/../trace/tests";
    CHECK(rpg::fs::pathExists(trace_dir_)) << trace_dir_;
  }

  rpg_common::Position org_;
  std::string trace_dir_;
};

TEST_F(DepthVoxelTest, testInit)
{
  DepthVoxel ds(org_, 2);
  std::cout << ds;
}

TEST_F(DepthVoxelTest, conversions)
{
  DepthVoxel ds(org_, 2);
  std::random_device rd;
  std::mt19937 gen(rd());

  // latitude longtitude to indices
  std::uniform_real_distribution<DepthVoxel::FloatType> lat_uni_deg(-90.0,
                                                                    90.0);
  std::uniform_real_distribution<DepthVoxel::FloatType> lon_uni_deg(-180.0,
                                                                    180.0);
  for (size_t i = 0; i < 50; i++)
  {
    DepthVoxel::FloatType lat = lat_uni_deg(gen);
    DepthVoxel::FloatType lon = lon_uni_deg(gen);

    int ridx = ds.latitudeDegToIdx(lat);
    EXPECT_NEAR(ds.idxToLatitudeDeg(ridx), lat, ds.getStepDeg());
    int cidx = ds.longtitudeDegToIdx(lon);
    EXPECT_NEAR(ds.idxToLongtitude(cidx), lon, ds.getStepDeg());
  }

  for (size_t i = 0; i < 50; i++)
  {
    DepthVoxel::Vec3 vec_w;
    vec_w.setRandom();

    DepthVoxel::FloatType lat, lon;
    std::tie(lat, lon) = ds.vecWToLatiLongti(vec_w);
    DepthVoxel::Vec3 vec_w_r;
    ds.latiLongtiToVecW(lat, lon, &vec_w_r);
    EXPECT_GT(vec_w.normalized().dot(vec_w_r.normalized()), 1e-4);

    int ridx, cidx;
    std::tie(ridx, cidx) = ds.vecWorldToDepthMapIndices(vec_w);
    ds.depthMapIndicesToVecWorld(ridx, cidx, &vec_w_r);
    EXPECT_GT(vec_w.normalized().dot(vec_w_r.normalized()),
              std::cos(ds.getStepDeg() * M_PI / 180.0));
  }
}

TEST_F(DepthVoxelTest, testQuery)
{
  DepthVoxel ds(org_, 2);

  Eigen::Vector3d pw;
  for (size_t i = 0; i < 100; i++)
  {
    pw = Eigen::Vector3d::Random();
    float d = ds.queryDepthWorldPoint(pw);
    EXPECT_EQ(d, DepthVoxel::kInfD);
  }

  const DepthVoxel::FloatType const_val(5.0);
  ds.setDepthMap(const_val);
  for (size_t i = 0; i < 100; i++)
  {
    pw = Eigen::Vector3d::Random();
    float d = ds.queryDepthWorldPoint(pw);
    EXPECT_EQ(d, const_val);
  }
}

TEST_F(DepthVoxelTest, setFromDepthImages)
{
  rpg_common::Pose Tbc;
  rpg_common::Pose Twb;
  Tbc.setIdentity();
  Twb.setIdentity();
  vi_utils::PinholeCamPtr cam_ptr = std::make_shared<vi_utils::PinholeCam>(
      std::vector<double>{ 320.0, 320.0, 321.5, 321.5, 640.0, 640.0 }, Tbc);

  DepthVoxel ds(org_, 2);
  EXPECT_EQ(ds.numValidDepths(), 0u);

  cv::Mat dimg;
  const DepthVoxel::FloatType const_d = 10.0;
  dimg.create(640.0, 640.0, CV_32F);
  dimg.setTo(const_d);
  ds.setFromDepthImages({ cam_ptr }, { dimg }, { Twb });
  EXPECT_GT(ds.numValidDepths(), 0u);
  std::cout << "Has " << ds.numValidDepths() << " valid depths.\n";

  Eigen::Vector3d d_same_near(0.5, 0.5, 5.0);
  Eigen::Vector3d d_same_center(0.0, 0.0, 5.0);
  Eigen::Vector3d d_opp(0.5, 0.5, -5.0);
  EXPECT_NEAR(const_d, ds.queryDepthWorldPoint(d_same_center), 1e-2);
  EXPECT_LT(const_d, ds.queryDepthWorldPoint(d_same_near));
  EXPECT_EQ(DepthVoxel::kInfD, ds.queryDepthWorldPoint(d_opp));
}

TEST_F(DepthVoxelTest, testSetResetQuery)
{
  DepthVoxel ds(org_, 2);
  const Eigen::Vector3d pw_far(10.0, 10.0, 10.0);
  const Eigen::Vector3d pw_near(1.0, 1.0, 1.0);

  rpg_common::PositionVec inline_pts{ pw_far, pw_near };

  const Eigen::Vector3d pw_else(1.0, 2.0, 3.0);

  EXPECT_EQ(ds.numValidDepths(), 0u);
  for (const Eigen::Vector3d& set_pt : inline_pts)
  {
    DepthVoxel::FloatType d_gt =
        static_cast<DepthVoxel::FloatType>((set_pt - org_).norm());
    ds.setDepthFromWorldPoint(set_pt);

    for (const Eigen::Vector3d& query_pt : inline_pts)
    {
      DepthVoxel::FloatType d_query = ds.queryDepthWorldPoint(query_pt);
      EXPECT_EQ(d_query, d_gt);
    }
    EXPECT_EQ(ds.queryDepthWorldPoint(pw_else), DepthVoxel::kInfD);
  }
  EXPECT_EQ(ds.numValidDepths(), 1u);

  ds.resetDepthmap();
  EXPECT_EQ(ds.numValidDepths(), 0u);
  for (const Eigen::Vector3d& pt : { pw_far, pw_near, pw_else })
  {
    EXPECT_EQ(ds.queryDepthWorldPoint(pt), DepthVoxel::kInfD);
  }
}

TEST_F(DepthVoxelTest, testCoverage)
{
  int img_dim = 640;
  // camera and depth image
  rpg_common::Pose Tbc;
  Tbc.setIdentity();
  vi_utils::PinholeCamPtr cam_ptr = std::make_shared<vi_utils::PinholeCam>(
      std::vector<double>{
          static_cast<double>(img_dim) / 2, static_cast<double>(img_dim) / 2,
          static_cast<double>(img_dim) / 2 - 0.5,
          static_cast<double>(img_dim) / 2 - 0.5, static_cast<double>(img_dim),
          static_cast<double>(img_dim) },
      Tbc);

  cv::Mat dimg;
  const DepthVoxel::FloatType const_d = 10.0;
  dimg.create(img_dim, img_dim, CV_32F);
  dimg.setTo(const_d);

  rpg_common::Pose Twc;
  Twc.setIdentity();

  //
  rpg_common::PoseVec Tws_vec;
  DepthVoxel::generateSampleTws(Twc, true, &Tws_vec);
  vi_utils::PinholeCamVec cam_vec(Tws_vec.size(), cam_ptr);
  std::vector<cv::Mat> dimg_vec;
  for (size_t i = 0; i < Tws_vec.size(); i++)
  {
    dimg_vec.push_back(dimg.clone());
  }

  //
  DepthVoxel ds(org_, 5.0);
  EXPECT_EQ(0u, ds.numValidDepths());
  EXPECT_EQ(ds.numTotalDepths(), ds.numValidDepths() + ds.numMissingDepths());
  EXPECT_EQ(VisStatus::kUnknownDepth,
            ds.queryVisibilityPoint(Eigen::Vector3d::Random()));
  EXPECT_EQ(false, ds.isPointOccluded(Eigen::Vector3d::Random()));
  ds.setFromDepthImages(cam_vec, dimg_vec, Tws_vec);
  EXPECT_EQ(ds.numTotalDepths(), ds.numValidDepths());
  EXPECT_EQ(0u, ds.numMissingDepths());

  ds.printMinMaxDepthMap();

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> in_dis(0.1f * const_d, 0.9f * const_d);
  // the farthest dist. appear at the corner of the image, thus sqrt(3)
  std::uniform_real_distribution<float> out_dis(1.74f * const_d,
                                                2.5f * const_d);
  for (size_t idx = 0; idx < 100; idx++)
  {
    Eigen::Vector3d rpt;
    rpt.setRandom();
    rpt.normalize();
    rpt *= static_cast<double>(in_dis(gen));
    EXPECT_EQ(VisStatus::kVisibile, ds.queryVisibilityPoint(rpt));
    EXPECT_FALSE(ds.isPointOccluded(rpt));

    rpt.setRandom();
    rpt.normalize();
    rpt *= static_cast<double>(out_dis(gen));
    EXPECT_EQ(VisStatus::kOccluded, ds.queryVisibilityPoint(rpt));
    EXPECT_TRUE(ds.isPointOccluded(rpt)) << rpt << " norm: " << rpt.norm();
  }
}

TEST_F(DepthVoxelTest, queryDepthImage)
{
  int img_dim = 640;
  rpg_common::Pose Tbc;
  Tbc.setIdentity();
  vi_utils::PinholeCamPtr cam_ptr = std::make_shared<vi_utils::PinholeCam>(
      std::vector<double>{
          static_cast<double>(img_dim) / 2, static_cast<double>(img_dim) / 2,
          static_cast<double>(img_dim) / 2 - 0.5,
          static_cast<double>(img_dim) / 2 - 0.5, static_cast<double>(img_dim),
          static_cast<double>(img_dim) },
      Tbc);

  cv::Mat dimg;
  const DepthVoxel::FloatType const_d = 10.0;
  dimg.create(img_dim, img_dim, CV_32F);
  dimg.setTo(const_d);

  rpg_common::Pose Twc;
  Twc.setIdentity();

  //
  rpg_common::PoseVec Tws_vec;
  DepthVoxel::generateSampleTws(Twc, true, &Tws_vec);
  vi_utils::PinholeCamVec cam_vec(Tws_vec.size(), cam_ptr);
  std::vector<cv::Mat> dimg_vec;
  for (size_t i = 0; i < Tws_vec.size(); i++)
  {
    dimg_vec.push_back(dimg.clone());
  }

  DepthVoxel ds(org_, 1.0);
  ds.setDepthMap(const_d);
  ds.setFromDepthImages(cam_vec, dimg_vec, Tws_vec);

  // offset
  rpg_common::Pose Twq;
  Twq.getRotation() = rpg::Rotation(Eigen::Vector3d(0.0, 0.25 * M_PI, 0.0));
  rpg::PoseVec Tws_q_vec;
  DepthVoxel::generateSampleTws(Twq, true, &Tws_q_vec);
  std::vector<cv::Mat> qimg_vec;
  ds.queryDepthImage(cam_vec, Tws_q_vec, &qimg_vec);

  for (size_t i = 0; i < qimg_vec.size(); i++)
  {
    std::ostringstream ss;
    ss << i << ".png";
    std::cout << "saving " << ss.str() << std::endl;
    cv::normalize(qimg_vec[i], qimg_vec[i], 0.0, 255.0, cv::NORM_MINMAX);
    cv::imwrite(trace_dir_ + "/" + ss.str(), qimg_vec[i]);
  }
}

TEST_F(DepthVoxelTest, serialization)
{
  DepthVoxel dv(org_, 2.0);
  dv.setDepthMap(10.0);
  EXPECT_EQ(0u, dv.numMissingDepths());

  {
    std::vector<uint32_t> buf32;
    dv.serializeToIntegers(&buf32);
    EXPECT_EQ(dv.serializationSize(), buf32.size());
    DepthVoxel dvl;
    dvl.deserializeFromIntegers(buf32, 0);
    EXPECT_TRUE(dvl.rayDepthMatCRef().isApprox(dv.rayDepthMatCRef()));
    EXPECT_TRUE(dvl.center().isApprox(dv.center()));
    EXPECT_DOUBLE_EQ(dvl.getStepDeg(), dv.getStepDeg());
  }

  {
    std::vector<uint64_t> buf64;
    dv.serializeToIntegers(&buf64);
    EXPECT_EQ(dv.serializationSize(), buf64.size());
    DepthVoxel dvl;
    dvl.deserializeFromIntegers(buf64, 0);
    EXPECT_TRUE(dvl.rayDepthMatCRef().isApprox(dv.rayDepthMatCRef()));
    EXPECT_TRUE(dvl.center().isApprox(dv.center()));
    EXPECT_DOUBLE_EQ(dvl.getStepDeg(), dv.getStepDeg());
  }
}

RPG_COMMON_TEST_MAIN
{
}
