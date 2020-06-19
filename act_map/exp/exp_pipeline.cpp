//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map/simulator.h"
#include "act_map/act_map.h"

#include <opencv2/opencv.hpp>

#include <rpg_common/main.h>

DEFINE_string(abs_traj, "", "Absolute path of the trajectory file.");
DEFINE_string(abs_map, "", "Absolute path of the map file.");
DEFINE_string(abs_trace_dir, "", "Absolute path of the trace directory.");

DEFINE_int32(min_obs,
             5,
             "Number of observations to make a landmark well "
             "observed.");

using namespace act_map;

namespace
{
void drawKeypoints(const Eigen::Matrix2Xd &us,
                   const cv::Scalar color,
                   cv::Mat *img)
{
  CHECK_NOTNULL(img);
  CHECK(!img->empty());

  for (int i = 0; i < us.cols(); i++)
  {
    cv::circle(*img, cv::Point2d(us(0, i), us(1, i)), 2, color);
  }
}
}

RPG_COMMON_MAIN
{
  std::cout << "This is to simulate the output of a SLAM system.\n"
               "The system, at each frame, output a sparse 3D map containing "
               "the well-observed/triangulated points.\n"
               "The output is fed into a volumetric mapping method to get "
               "a map representation for active SLAM.\n";

  CHECK(!FLAGS_abs_traj.empty());
  CHECK(!FLAGS_abs_map.empty());

  std::cout << "Experiment parameters:\n"
            << "- traj: " << FLAGS_abs_traj << std::endl
            << "- map: " << FLAGS_abs_map << std::endl
            << "- trace: " << FLAGS_abs_trace_dir << std::endl
            << "- min_obs: " << FLAGS_min_obs << std::endl;

  vi::StatesVec states_vec;
  vi::States::load(FLAGS_abs_traj, &states_vec, ',', true);
  vi::MapPtr map = std::make_shared<vi::Map>();
  map->load(FLAGS_abs_map, "");

  std::vector<double> cam_geo{ 300, 300, 300, 300, 600, 600 };
  rpg::Pose Tbc;
  Tbc.setIdentity();
  Eigen::Matrix3d rot_mat;
  rot_mat << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  rpg::Rotation rot(rot_mat);
  Tbc.getRotation() = rot;
  vi::PinholeCamPtr cam = std::make_shared<vi::PinholeCam>(cam_geo, Tbc);
  vi::PinholeCamVec cams;
  cams.push_back(cam);

  Simulator sim(states_vec, map, cams);
  std::cout << "The simulator is " << sim;
  sim.initSequentialSim(FLAGS_min_obs);

  ActMapOptions am_options;
  TraceMap am(am_options);

  const std::string wn("obs");
  cv::namedWindow(wn);
  const size_t viz_cam = 0;
  for (size_t states_idx = 0; states_idx < sim.numOfStates(); states_idx++)
  {
    std::cout << "Simulation step: " << states_idx + 1 << "/"
              << sim.numOfStates() << std::endl;
    sim.step();
    vi::CamMeasurementsVec meas_i;
    sim.getLastObservations(&meas_i, nullptr);
    Mat3XdVec good_pts_w;
    PointIdsVec good_ids;
    Mat2XdVec good_us;
    sim.getLastWellObserved(&good_pts_w, &good_ids, &good_us);
    vi::States states_i;
    sim.getLastStates(&states_i);
    rpg::Pose T_w_b = states_i.T_0_cur;

    for (size_t cam_idx = 0; cam_idx < sim.numOfCams(); cam_idx++)
    {
      const int n_good_pts_i = good_pts_w[cam_idx].cols();
      std::cout << "Number of good points is " << n_good_pts_i << std::endl;
      if (n_good_pts_i == 0)
      {
        continue;
      }
      rpg::Pose T_w_ci = T_w_b * sim.getCamConstRef(cam_idx).Tbc();
      Eigen::Matrix3Xd points_c =
          T_w_ci.inverse().transformVectorized(good_pts_w[cam_idx]);
      am.integratePointCloudOccupancy(T_w_ci, points_c);
    }

    cv::Mat img;
    img.create(static_cast<int>(cam->w()), static_cast<int>(cam->h()), CV_8UC3);
    img.setTo(cv::Scalar(120, 120, 120));
    drawKeypoints(meas_i[viz_cam].keypoints, cv::Scalar(255, 0, 0), &img);
    drawKeypoints(good_us[viz_cam], cv::Scalar(0, 255, 0), &img);
    cv::imshow(wn, img);

    cv::waitKey(100);  // ms
  }

  return 0;
}
