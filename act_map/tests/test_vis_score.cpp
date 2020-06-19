//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map/vis_score.h"

#include <cmath>

#include <vi_utils/cam_min.h>
#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>

class VisScoreTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    rpg::Pose Tbc;
    Tbc.setIdentity();

    double width = 640.0;
    double height = 480.0;
    std::vector<double> geo_params{ width / 2,  height / 2, width / 2,
                                    height / 2, width,      height };
    cam_ = std::make_shared<vi::PinholeCam>(geo_params, Tbc);
    hfov_rad_ = M_PI_4;

    std::string dir;
    std::string fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    abs_cfg_fn_ = dir + "/test_data/vis_score.txt";

  }
  vi::PinholeCamPtr cam_;
  double hfov_rad_;
  double boundary_to_mid_ratio_ = 0.9;
  double boundary_value_ = 0.9;

  std::string abs_cfg_fn_;
};

TEST_F(VisScoreTest, testInitializationAndQuery)
{
  act_map::VisScore vscore(hfov_rad_);
  EXPECT_DOUBLE_EQ(1.0, vscore.exactVisibility(std::cos(hfov_rad_ * 0.9)));
  EXPECT_DOUBLE_EQ(1.0, vscore.exactVisibility(std::cos(hfov_rad_ * 0.2)));
  EXPECT_DOUBLE_EQ(0.0, vscore.exactVisibility(std::cos(hfov_rad_ * 1.1)));
  EXPECT_DOUBLE_EQ(0.0, vscore.exactVisibility(std::cos(hfov_rad_ * 1.5)));
}

TEST_F(VisScoreTest, testQueryBearing)
{
  act_map::VisScore vscore(hfov_rad_);
  for (int i = 0; i < 50; i++)
  {
    Eigen::Vector3d f;
    f.setRandom();
    Eigen::Vector3d uf = f.normalized();
    double cosf = uf.dot(Eigen::Vector3d(0, 0, 1));
    EXPECT_DOUBLE_EQ(vscore.exactVisibility(cosf), vscore.exactVisibility(f));
  }
}

TEST_F(VisScoreTest, testApproxInit)
{
  {
    act_map::VisScore vscore(hfov_rad_);
    EXPECT_FALSE(vscore.secondOrderApproxInitialized());
    vscore.initSecondOrderApprox(boundary_to_mid_ratio_, boundary_value_);
    EXPECT_NEAR(-0.0928932188135, vscore.k1(), 1e-5);
    EXPECT_NEAR(0.5, vscore.k2(), 1e-5);
    EXPECT_NEAR(0.592893218813, vscore.k3(), 1e-5);
    EXPECT_TRUE(vscore.secondOrderApproxInitialized());
  }

  // with options
  {
    act_map::VisScoreOptions options;
    options.half_fov_rad = hfov_rad_;
    options.boundary_to_mid_ratio = boundary_to_mid_ratio_;
    options.boundary_value = boundary_value_;
    act_map::VisScore vscore2(options);
    EXPECT_TRUE(vscore2.secondOrderApproxInitialized());
    EXPECT_NEAR(-0.0928932188135, vscore2.k1(), 1e-5);
    EXPECT_NEAR(0.5, vscore2.k2(), 1e-5);
    EXPECT_NEAR(0.592893218813, vscore2.k3(), 1e-5);
  }

  {
    act_map::VisScore vscore3(M_PI / 2.5);
    vscore3.initSecondOrderApprox(boundary_to_mid_ratio_, boundary_value_);
    EXPECT_NEAR(-0.27140873035, vscore3.k1(), 1e-5);
    EXPECT_NEAR(0.5, vscore3.k2(), 1e-5);
    EXPECT_NEAR(0.77140873035, vscore3.k3(), 1e-5);
    EXPECT_TRUE(vscore3.secondOrderApproxInitialized());
  }
}

TEST_F(VisScoreTest, test2ndOrderApprox)
{
  act_map::VisScore vscore(hfov_rad_);
  EXPECT_FALSE(vscore.secondOrderApproxInitialized());
  vscore.initSecondOrderApprox(boundary_to_mid_ratio_, boundary_value_);

  Eigen::Vector3d center(0, 0, 1);
  Eigen::Vector3d back(0, 0, -1);
  Eigen::Vector3d bound1(0, std::tan(hfov_rad_), 1);
  Eigen::Vector3d bound2(std::tan(hfov_rad_), 0, 1);

  EXPECT_DOUBLE_EQ(1.0, vscore.secondOrderVisibility(center));
  EXPECT_DOUBLE_EQ(0.0, vscore.secondOrderVisibility(back));
  EXPECT_DOUBLE_EQ(boundary_value_, vscore.secondOrderVisibility(bound1));
  EXPECT_DOUBLE_EQ(boundary_value_, vscore.secondOrderVisibility(bound2));
}

TEST_F(VisScoreTest, testLoadAndSamples)
{
  act_map::VisScorePtr vis_score_ptr = act_map::VisScore::load(abs_cfg_fn_);
  EXPECT_TRUE(vis_score_ptr.get() != nullptr);
  EXPECT_TRUE(vis_score_ptr->secondOrderApproxInitialized());

  std::vector<double> samples;
  vis_score_ptr->createSamples(10.0, &samples);
  for (const double v : samples)
  {
    EXPECT_LE(v, 1.0);
    EXPECT_GE(v, 0.0);
  }
}

RPG_COMMON_TEST_MAIN
{
}
