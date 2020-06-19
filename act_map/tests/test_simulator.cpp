//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map/simulator.h"

#include <string>
#include <random>
#include <ctime>

#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>

using namespace act_map;

namespace
{
size_t getRandomIdx(const size_t size)
{
  std::srand(std::time(nullptr));
  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_int_distribution<size_t> idx_dis(0, size - 1);
  return idx_dis(rng);
}
}

class SimulatorTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::vector<double> cam_geo{ 300, 300, 300, 300, 600, 600 };

    rpg::Pose Tbc;
    Tbc.setIdentity();
    Eigen::Matrix3d rot_mat;
    rot_mat << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    rpg::Rotation rot(rot_mat);
    Tbc.getRotation() = rot;

    vi::PinholeCamPtr cam = std::make_shared<vi::PinholeCam>(cam_geo, Tbc);
    cams_.push_back(cam);

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
  vi::PinholeCamVec cams_;
};

TEST_F(SimulatorTest, testInitialization)
{
  Simulator sim1;
  EXPECT_FALSE(sim1.isInitialized());

  sim1.initialize(states_, map_, cams_);
  EXPECT_TRUE(sim1.isInitialized());

  Simulator sim2(states_, map_, cams_);
  EXPECT_TRUE(sim2.isInitialized());

  std::cout << sim1;
  std::cout << sim2;
}

TEST_F(SimulatorTest, testMisc)
{
  Simulator sim(states_, map_, cams_);
  EXPECT_EQ(sim.numOfCams(), cams_.size());
  EXPECT_EQ(sim.numOfStates(), states_.size());

  vi::States states_i;
  size_t test_id = getRandomIdx(sim.numOfStates());
  sim.getStatesAt(test_id, &states_i);
  EXPECT_EQ(states_i.time_ns, states_[test_id].time_ns);
}

TEST_F(SimulatorTest, SingleSim)
{
  Simulator sim(states_, map_, cams_);

  size_t test_id = getRandomIdx(sim.numOfStates());
  vi::States test_states;
  sim.getStatesAt(test_id, &test_states);

  vi::CamMeasurementsVec cam_meas;
  sim.getObservationAt(test_id, &cam_meas);
  Mat3XdVec pts_vec;
  sim.getObservationAt(test_id, &cam_meas, &pts_vec);

  EXPECT_EQ(cam_meas.size(), sim.numOfCams());

  for (size_t cam_idx = 0; cam_idx < cams_.size(); cam_idx++)
  {
    const vi::PinholeCam& cam_i = sim.getCamConstRef(cam_idx);
    const vi::CamMeasurements& meas_i = cam_meas[cam_idx];
    const Eigen::Matrix3Xd points_w_i = pts_vec[cam_idx];
    for (size_t obs_idx = 0; obs_idx < meas_i.global_lm_ids.size(); obs_idx++)
    {
      const Eigen::Vector3d pw =
          map_->points_.col(meas_i.global_lm_ids[obs_idx]);
      EXPECT_TRUE(pw.isApprox(points_w_i.col(obs_idx)));
      const Eigen::Vector3d pc =
          (test_states.T_0_cur * cam_i.Tbc()).inverse().transform(pw);
      Eigen::Vector2d u;
      EXPECT_TRUE(cam_i.project3d(pc, &u));
      EXPECT_TRUE(u.isApprox(meas_i.keypoints.col(static_cast<int>(obs_idx))));
    }
  }
}

TEST_F(SimulatorTest, SequentialSim)
{
  Simulator sim(states_, map_, cams_);

  sim.initSequentialSim();

  EXPECT_EQ(sim.nextStateIdx(), 0u);
  EXPECT_TRUE(sim.lastMeas().empty());
  EXPECT_TRUE(sim.lastPoints3D().empty());
  EXPECT_TRUE(sim.getPtIdToNumObsMap().empty());

  sim.step();

  EXPECT_EQ(sim.nextStateIdx(), 1u);
  EXPECT_FALSE(sim.lastMeas().empty());
  EXPECT_FALSE(sim.lastPoints3D().empty());
  EXPECT_EQ(sim.lastMeas()[0].timestamp_ns, states_[0].time_ns);
  EXPECT_FALSE(sim.getPtIdToNumObsMap().empty());

  for (auto v : sim.getPtIdToNumObsMap())
  {
    EXPECT_EQ(1, v.second);
  }

  sim.step();
  EXPECT_EQ(sim.nextStateIdx(), 2u);

  for (auto v : sim.getPtIdToNumObsMap())
  {
    EXPECT_TRUE(1 == v.second || 2 == v.second);
  }

  sim.initSequentialSim();
  EXPECT_EQ(sim.nextStateIdx(), 0u);
  EXPECT_TRUE(sim.lastMeas().empty());
  EXPECT_TRUE(sim.lastPoints3D().empty());
  EXPECT_TRUE(sim.getPtIdToNumObsMap().empty());
}

TEST_F(SimulatorTest, testSimOutput)
{
  Simulator sim(states_, map_, cams_);
  sim.initSequentialSim(3);

  sim.step();

  Eigen::Matrix3Xd all_good_pts;
  std::vector<int> all_good_ids;
  sim.getLastWellObserved(&all_good_pts, &all_good_ids);

  Mat3XdVec good_points_mat;
  PointIdsVec good_ids;
  Mat2XdVec good_us_mat;
  sim.getLastWellObserved(&good_points_mat, &good_ids, &good_us_mat);

  V3dVecVec good_points_vec;
  V2dVecVec good_us_vec;
  sim.getLastWellObserved(&good_points_vec, &good_ids, &good_us_vec);


  EXPECT_EQ(all_good_pts.cols(), 0);
  sim.step();
  sim.step();
  sim.getLastWellObserved(&all_good_pts, &all_good_ids);

  EXPECT_GT(all_good_pts.cols(), 0);
  EXPECT_EQ(all_good_pts.cols(), all_good_ids.size());

  for (size_t i = 0; i < all_good_ids.size(); i++)
  {
    const Eigen::Vector3d& pt = map_->points_.col(all_good_ids[i]);
    EXPECT_TRUE(pt.isApprox(all_good_pts.col(static_cast<int>(i))));
  }
}

RPG_COMMON_TEST_MAIN
{
}
