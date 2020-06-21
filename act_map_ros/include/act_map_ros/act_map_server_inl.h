#pragma once

#include "act_map_ros/act_map_server.h"

#include <rpg_common/fs.h>
#include <rpg_common_ros/params_helper.h>
#include <rpg_common_ros/tf.h>
#include <tf/transform_listener.h>

#include <voxblox_ros/ptcloud_vis.h>
#include <act_map/voxblox_utils.h>

#include "act_map_ros/conversion_ros.h"
#include "act_map_ros/params_reader.h"

namespace act_map_ros
{
template <typename T>
ActMapServer<T>::ActMapServer(const ros::NodeHandle& nh,
                              const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  readParams();
  init();
  setupROS();
}

template <typename T>
void ActMapServer<T>::readServerOptions()
{
  options_.n_cams_ = static_cast<size_t>(rpg_ros::param(pnh_, "n_cams", 1));
  options_.viz_occupancy_ = rpg_ros::param(pnh_, "viz_occupancy", true);
  options_.features3d_topic_name_ =
      rpg_ros::param<std::string>(pnh_, "features3d_topic_name", "features3d");
  options_.body_pose_topic_name_ =
      rpg_ros::param<std::string>(pnh_, "body_pose_topic_name", "Twb");
  options_.cam_frame_names_.resize(options_.n_cams_);
  for (size_t i = 0; i < options_.n_cams_; i++)
  {
    options_.cam_frame_names_[i] =
        rpg_ros::param<std::string>(pnh_, "cam_frame" + std::to_string(i), "");
  }

  options_.preset_kernel_layer_ =
      rpg_ros::param(pnh_, "preset_kernel_layer", false);
  if (options_.preset_kernel_layer_)
  {
    XmlRpc::XmlRpcValue v;
    CHECK(pnh_.getParam("preset_kernel_layer_ranges", v));
    VLOG(1) << "Preset kernel layer range: ";
    for (int i = 0; i < v.size(); i++)
    {
      options_.preset_ker_layer_ranges_.push_back(v[i]);
      VLOG(1) << v[i];
    }
  }
  std::string kupdate =
      rpg_ros::param<std::string>(pnh_, "ker_update_mode", "batch");
  options_.setKernelUpdateMode(kupdate);
  options_.batch_recompute_occ_thresh_ =
      rpg_ros::param(pnh_, "batch_recompute_occ_thresh", 0.2);
  XmlRpc::XmlRpcValue v;
  CHECK(pnh_.getParam("kernel_expand_ranges", v));
  VLOG(1) << "Found kernel expand ranges: ";
  for (int i = 0; i < v.size(); i++)
  {
    options_.kernel_expand_ranges_.push_back(v[i]);
    VLOG(1) << v[i];
  }
  options_.only_activate_nearby_kernel_blks_ =
      rpg_ros::param(pnh_, "only_activate_nearby_kernel_blks", false);
  options_.kernel_blks_activate_dist_ =
      rpg_ros::param(pnh_, "kernel_blks_activate_dist", 3.0);
  options_.kernel_expand_dist_thresh_ =
      rpg_ros::param(pnh_, "kernel_expand_dist_thresh", 1.0);

  options_.viz_allocated_kernel_blks_ =
      rpg_ros::param(pnh_, "viz_allocated_kernel_blk", false);
  options_.viz_active_kernel_blks_ =
      rpg_ros::param(pnh_, "viz_active_kernel_blk", false);
  options_.viz_kvox_best_view_ =
      rpg_ros::param(pnh_, "viz_kvox_best_view", false);
  options_.viz_bview_samples_per_side_ =
      rpg_ros::param(pnh_, "viz_kvox_bview_samples_per_side", 3);
  options_.viz_bview_use_sampling_ =
      rpg_ros::param(pnh_, "viz_bview_use_sampling", false);
  options_.viz_bview_incremental_every_n_ =
      rpg_ros::param(pnh_, "viz_bview_incremental_every_n", 2);
  options_.viz_bview_last_updated_ =
      rpg_ros::param(pnh_, "viz_bview_last_updated", true);
  options_.viz_bview_size_ = rpg_ros::param(pnh_, "viz_bview_size", 0.1);
  options_.viz_bview_use_unique_id_ =
      rpg_ros::param(pnh_, "viz_bview_use_unique_id", true);
  options_.viz_cam_marker_size_ =
      rpg_ros::param(pnh_, "viz_cam_marker_size", 0.2);
  options_.viz_bview_fixed_color_ =
      rpg_ros::param(pnh_, "viz_bview_fixed_color", true);
  options_.viz_bview_color_scale_log_n_ =
      rpg_ros::param(pnh_, "viz_bview_color_scale_log_n", 2);

  options_.save_dir_ = rpg_ros::param<std::string>(pnh_, "save_map_dir", "");
  if (options_.save_dir_.empty())
  {
    std::string dir;
    std::string fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    options_.save_dir_ = dir + "/../../trace";
  }
}

template <typename T>
void ActMapServer<T>::readParams()
{
  readServerOptions();
  map_options_.use_collision_checker_ =
      rpg_ros::param(pnh_, "use_collision_checker", false);
  map_options_.occ_layer_options_ = readLayerOptions(pnh_, "occ_");
  map_options_.occ_integrator_options_ = readOccupancyIntegratorConfig(pnh_);
  map_options_.vis_options_.resize(options_.n_cams_);
  for (size_t i = 0; i < options_.n_cams_; i++)
  {
    map_options_.vis_options_[i] = readVisScoreOptions(pnh_, std::to_string(i));
  }
  map_options_.ker_layer_options_ = readLayerOptions(pnh_, "ker_");
  map_options_.ker_integrator_options_ = readKernelIntegratorOptions(pnh_);
  map_options_.col_ops_ = readCollisionCheckerOptions(pnh_);
}

template <typename T>
void ActMapServer<T>::init()
{
  cam_frames_.resize(options_.n_cams_);
  for (size_t i = 0; i < options_.n_cams_; i++)
  {
    if (options_.cam_frame_names_[i].empty())
    {
      createCamFrameId(i, &cam_frames_[i]);
    }
    else
    {
      cam_frames_[i] = options_.cam_frame_names_[i];
    }
  }
  act_map_ =
      std::unique_ptr<act_map::ActMap<T>>(new act_map::ActMap<T>(map_options_));
  pub_kernel_blk_centers_ = false;
  pub_kvox_bestview_ = false;

  if (options_.preset_kernel_layer_)
  {
    act_map_->allocateKernelLayerUniform(options_.preset_ker_layer_ranges_);
    VLOG(1) << "Pre-allocated "
            << act_map_->kerLayerCRef().getNumberOfAllocatedBlocks()
            << " blocks";
    pub_kernel_blk_centers_ = true;
  }
  inc_step_cnt_ = 0;
}

template <typename T>
void ActMapServer<T>::setupROS()
{
  feature_3d_w_sub_.clear();
  for (size_t i = 0; i < options_.n_cams_; i++)
  {
    feature_3d_w_sub_.emplace_back(nh_.subscribe<PCLPointCloud>(
        options_.features3d_topic_name_ + std::to_string(i),
        10,
        boost::bind(&ActMapServer::feature3dCallback, this, _1, i)));
  }
  body_pose_sub_ =
      nh_.subscribe<geometry_msgs::PoseStamped>(options_.body_pose_topic_name_,
                                                10,
                                                &ActMapServer::bodyPoseCallback,
                                                this);

  occupied_vox_ = pnh_.advertise<PCLPointCloud>("occupied_vox", 5);
  kernel_blk_centers_pub_ =
      pnh_.advertise<PCLPointCloud>("kernel_blk_centers", 10);
  kvox_bestviews_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>(
      "kernel_vox_best_views", 10);
  pub_markers_ = pnh_.advertise<visualization_msgs::Marker>("markers", 10);

  key_cmd_sub_ = nh_.subscribe<std_msgs::String>(
      "/act_map_cmd", 10, &ActMapServer::keyCmdCallback, this);
}

template <typename T>
void ActMapServer<T>::feature3dCallback(const PCLPointCloud::ConstPtr& pc,
                                        const size_t cam_idx)
{
  ros::Time pc_time;
  pcl_conversions::fromPCL(pc->header.stamp, pc_time);

  if (pc->empty())
  {
    visualizeAll(pc_time);
    return;
  }

  CHECK(pc->header.frame_id == kWorldFrame);  // TODO: directly publish in cam

  rpg::Pose T_w_c;

  tf::TransformListener listener;
  listener.waitForTransform(
      kWorldFrame, cam_frames_[cam_idx], pc_time, ros::Duration(0.05));
  if (!rpg_ros::tf::get_T_A_B(
          kWorldFrame, cam_frames_[cam_idx], pc_time, &T_w_c))
  {
    LOG(WARNING) << "Could not find TF, not integrating.";
    return;
  }

  rpg::Timer timer;

  timer.start();
  Eigen::Matrix3Xd pc_mat_world;
  pCLPointCloudToEigen3Xd(*pc, &pc_mat_world);
  Eigen::Matrix3Xd pc_mat_cam;
  pc_mat_cam = T_w_c.inverse().transformVectorized(pc_mat_world);
  act_map_->integratePointCloudOccupancy(T_w_c, pc_mat_cam);
  VLOG(2) << "Updated occupancy layer. It took " << timer.stop() * 1000 << " m"
                                                                           "s";

  timer.start();
  updateKernelLayer();
  VLOG(2) << "Updated kernel layer. It took " << timer.stop() * 1000 << " ms";

  timer.start();
  visualizeAll(pc_time);
  VLOG(2) << "Visualization took " << timer.stop() * 1000 << " ms";
}

template <typename T>
void ActMapServer<T>::updateKernelLayer()
{
  if (options_.ker_update_mode_ == KernelUpdateMode::kBatch)
  {
    double occ_updated_ratio = 0.0;
    {
      occ_updated_ratio = act_map_->updatedBlkRatioOcc();
    }

    if (occ_updated_ratio < options_.batch_recompute_occ_thresh_)
    {
      return;
    }
    rpg::Timer timer;
    timer.start();
    act_map_->recomputeKernelLayer();
    VLOG(1) << "Recomputed kernel layer. It took " << timer.stop() * 1000
            << " ms";
    act_map::utils::clearLayerUpdate(act_map_->occLayerPtr().get());
    pub_kvox_bestview_ = true;
  }
  else if (options_.ker_update_mode_ == KernelUpdateMode::kIncremental)
  {
    act_map_->updateKernelLayerIncremental();
    if (inc_step_cnt_ % options_.viz_bview_incremental_every_n_ == 0)
    {
      pub_kvox_bestview_ = true;
    }
    inc_step_cnt_++;
  }
}

template <typename T>
void ActMapServer<T>::keyCmdCallback(const std_msgs::StringConstPtr& key)
{
  const std::string& data = key->data;
  if (options_.ker_update_mode_ == KernelUpdateMode::kOnDemand)
  {
    if (data == "d")
    {
      act_map_->kerLayerPtr()->removeAllBlocks();
      visualizeKerBestViews();
      VLOG(1) << "Clear all blocks in the kernel layer.";
    }
    else if (data == "l")
    {
      act_map_->allocateKernelLayerUniform(options_.preset_ker_layer_ranges_);
      VLOG(1) << "Allocated "
              << act_map_->kerLayerPtr()->getNumberOfAllocatedBlocks()
              << " blocks in the kernel layer.";
    }
    else if (data == "c")
    {
      rpg::Timer timer;
      timer.start();
      act_map_->recomputeKernelLayer();
      VLOG(1) << "Recomputed kernel layer with "
              << act_map_->kerLayerPtr()->getNumberOfAllocatedBlocks()
              << " blocks "
              << "took " << timer.stop() * 1000 << " ms.";
      visualizeKerBestViews();
    }
  }

  if (data == "s")
  {
    const std::string occ_fn = options_.save_dir_ + "/occ_layer.protobuf";
    const std::string ker_fn = options_.save_dir_ + "/" +
                               act_map::getVoxelType<T>() + "_layer.protobuf";

    act_map_->saveLayers(occ_fn, ker_fn);
  }
}

template <typename T>
void ActMapServer<T>::bodyPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose)
{
  static rpg::Pose prev_added_Twb;
  rpg::Pose Twb;
  rosPoseToKindr(pose->pose, &Twb);

  if ((Twb.getPosition() - prev_added_Twb.getPosition()).norm() >
      options_.kernel_expand_dist_thresh_)
  {
    rpg::Timer timer;
    timer.start();
    act_map_->addRegionToKernelLayer(Twb, options_.kernel_expand_ranges_);
    VLOG(2) << "Expand kernel layer took " << timer.stop() * 1000 << " ms";
    pub_kernel_blk_centers_ = true;

    if (options_.only_activate_nearby_kernel_blks_)
    {
      const double block_half_dia_len = act_map_->kerBlockHalfDiagonal();
      act_map_->activateBlocksByDistance(Twb.getPosition(),
                                         options_.kernel_blks_activate_dist_ +
                                             block_half_dia_len);
    }
    prev_added_Twb = Twb;
  }
}

template <typename T>
void ActMapServer<T>::visualizeAll(const ros::Time& time)
{
  publishCameraMarker(pub_markers_,
                      cam_frames_[0],
                      "cam",
                      time,
                      1,
                      visualization_msgs::Marker::ADD,
                      options_.viz_cam_marker_size_,
                      Eigen::Vector3d(0, 0, 1));

  if (options_.viz_occupancy_)
  {
    if (occupied_vox_.getNumSubscribers() > 0)
    {
      visualizeOccupancy();
    }
  }

  if (options_.viz_allocated_kernel_blks_)
  {
    if (kernel_blk_centers_pub_.getNumSubscribers() > 0 &&
        pub_kernel_blk_centers_)
    {
      visualizeKernelBlocks();
      pub_kernel_blk_centers_ = false;
    }
  }

  if (options_.viz_kvox_best_view_ && pub_kvox_bestview_ &&
      kvox_bestviews_pub_.getNumSubscribers() > 0)
  {
    visualizeKerBestViews();
    pub_kvox_bestview_ = false;
    // this should be the end of the pipeline, so we can clear it
    act_map_->clearAccumulatedUpdatedKernelBlockIndices();
  }
}

template <typename T>
void ActMapServer<T>::visualizeOccupancy() const
{
  act_map::Vec3dVec occ_vox_cs;
  act_map_->getCentersOfOccupiedVoxels(&occ_vox_cs);
  PCLPointCloud occ_pc;
  Vec3dVecToPCLPointCloud(occ_vox_cs, &occ_pc);
  setPCLPointCloud(&occ_pc, kWorldFrame, ros::Time().now());
  occupied_vox_.publish(occ_pc);
}

template <typename T>
void ActMapServer<T>::visualizeKernelBlocks() const
{
  act_map::Vec3dVec blk_cs;
  if (options_.viz_active_kernel_blks_)
  {
    act_map::utils::getCentersOfAllActivatedBlocks(act_map_->kerLayerCRef(),
                                                   &blk_cs);
  }
  else
  {
    act_map::utils::getCentersOfAllBlocks(act_map_->kerLayerCRef(), &blk_cs);
  }
  PCLPointCloud blk_pc;
  Vec3dVecToPCLPointCloud(blk_cs, &blk_pc);
  setPCLPointCloud(&blk_pc, kWorldFrame, ros::Time().now());
  kernel_blk_centers_pub_.publish(blk_pc);
}

template <typename T>
void ActMapServer<T>::visualizeKerBestViews() const
{
  if (act_map::getVoxelType<T>() != act_map::voxel_types::kTraceKernel)
  {
    return;
  }
  act_map::Vec3dVec vox_cs;
  act_map::Vec3dVec bviews;

  rpg::Timer timer;
  timer.start();
  voxblox::LongIndexVector global_idxs;
  std::vector<double> values;
  act_map_->getBestViewsAt(0,
                           options_.viz_bview_samples_per_side_,
                           options_.viz_bview_use_sampling_,
                           options_.viz_bview_last_updated_,
                           &vox_cs,
                           &bviews,
                           &values,
                           &global_idxs);
  VLOG(2) << "Computing " << vox_cs.size() << " views for visulization took "
          << timer.stop() * 1000 << " ms.";

  visualization_msgs::MarkerArray ma;
  std::vector<size_t> ids;
  if (options_.viz_bview_use_unique_id_)
  {
    voxblox::LongIndexHash hasher;
    for (const voxblox::LongIndex& idx : global_idxs)
    {
      ids.push_back(hasher(idx));
    }
  }
  else
  {
    for (size_t i = 0; i < global_idxs.size(); i++)
    {
      ids.push_back(i);
    }
  }

  std::vector<std_msgs::ColorRGBA> colors(vox_cs.size());
  if (!options_.viz_bview_fixed_color_ && vox_cs.size() > 10)
  {
    std::vector<double> intensities;
    scaleLog10Ntimes(values,
                     static_cast<size_t>(options_.viz_bview_color_scale_log_n_),
                     &intensities);
    normalizeToRGB(intensities, &colors);
  }
  else
  {
    for (size_t i = 0; i < colors.size(); i++)
    {
      colors[i].a = 0.8f;
      colors[i].g = 0.5f;
    }
  }
  directionsToArrowsArray(
      vox_cs, bviews, ids, colors, options_.viz_bview_size_, &ma);
  kvox_bestviews_pub_.publish(ma);
}
}
