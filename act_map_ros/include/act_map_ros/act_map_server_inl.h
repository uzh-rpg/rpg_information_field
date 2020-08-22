#pragma once

#include "act_map_ros/act_map_server.h"

#include <numeric>

#include <rpg_common/fs.h>
#include <rpg_common/load.h>
#include <rpg_common_ros/params_helper.h>
#include <rpg_common_ros/tf.h>
#include <vi_utils/map.h>
#include <tf/transform_listener.h>

#include <act_map/voxblox_utils.h>

#include "act_map_ros/voxblox_ros/ptcloud_vis.h"
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

  // kernel layer
  options_.preset_kernel_layer_ =
      rpg_ros::param(pnh_, "preset_kernel_layer", false);
  if (options_.preset_kernel_layer_)
  {
    XmlRpc::XmlRpcValue v;
    CHECK(pnh_.getParam("preset_kernel_layer_ranges", v));
    VLOG(1) << "Preset kernel layer range: ";
    for (int i = 0; i < v.size(); i++)
    {
      options_.preset_pos_factor_layer_ranges_.push_back(v[i]);
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

  // occupancy layer
  options_.preset_occ_layer_ = rpg_ros::param(pnh_, "preset_occ_layer", false);
  options_.occ_points_fn_ =
      rpg_ros::param<std::string>(pnh_, "occ_points_fn", "");
  options_.aver_view_dirs_fn_ =
      rpg_ros::param<std::string>(pnh_, "aver_views_fn", "");

  // visualization
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
  options_.viz_bview_size_vox_ratio_ =
      rpg_ros::param(pnh_, "viz_bview_size_vox_ratio", -1.0);
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
  this->readServerOptions();
  map_options_ = readActMapOptions(pnh_, options_.n_cams_);
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

  // set occupancy
  if (options_.preset_occ_layer_)
  {
    CHECK(rpg::fs::fileExists(options_.occ_points_fn_))
        << options_.occ_points_fn_ << " does not exist.";

    vi::Map map;
    map.load(options_.occ_points_fn_, std::string());
    VLOG(1) << "Loaded " << map.n_points_ << " points.";

    Eigen::Matrix3Xd view_dirs;
    if (!options_.aver_view_dirs_fn_.empty())
    {
      CHECK(rpg::fs::fileExists(options_.aver_view_dirs_fn_))
          << options_.aver_view_dirs_fn_ << " does not exist.";

      Eigen::MatrixX3d loaded_view_dirs;
      rpg::load(options_.aver_view_dirs_fn_, &loaded_view_dirs);
      VLOG(1) << "Loaded " << loaded_view_dirs.rows() << " view directions.";
      view_dirs = loaded_view_dirs.transpose();
    }

    VLOG(1) << "Setting points in the map...";
    act_map_->setOccupancyWorldPoints(map.points_, view_dirs);
    CHECK_GE(map.n_points_, act_map_->numOccupiedVoxels());
    VLOG(1) << act_map_->numOccupiedVoxels() << " out of "
            << act_map_->occLayerPtr()->getNumberOfAllocatedVoxels()
            << " are occupied.";
  }

  // need to call before possible allocation
  typedInit();

  // allocate kernel layer in a given range
  if (options_.preset_kernel_layer_)
  {
    act_map_->allocateFactorLayerUniform(
        options_.preset_pos_factor_layer_ranges_);
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
        options_.features3d_topic_name_ + std::to_string(i), 10,
        boost::bind(&ActMapServer::feature3dCallback, this, _1, i)));
  }
  body_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      options_.body_pose_topic_name_, 10, &ActMapServer::bodyPoseCallback,
      this);

  occupied_vox_ = pnh_.advertise<PCLPointCloud>("occupied_vox", 5);
  occ_view_dir_pub_ =
      pnh_.advertise<visualization_msgs::MarkerArray>("occupied_vox_views", 5);
  kernel_blk_centers_pub_ =
      pnh_.advertise<PCLPointCloud>("kernel_blk_centers", 10);
  kvox_bestviews_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("kernel"
                                                                        "_vox_"
                                                                        "best_"
                                                                        "views",
                                                                        10);
  pub_markers_ = pnh_.advertise<visualization_msgs::MarkerArray>("markers", 10);

  // services
  update_vis_srv_ = pnh_.advertiseService(
      "update_act_map_viz", &ActMapServer::updateVisualizationCallback, this);
  clear_map_srv_ = pnh_.advertiseService("clear_act_map_layers",
                                         &ActMapServer::clearMapCallback, this);
  recompute_srv_ = pnh_.advertiseService(
      "recompute_kernel_layer", &ActMapServer::recomputeCallBack, this);
  allocate_ker_map_srv_ = pnh_.advertiseService(
      "alloc_kernel_layer", &ActMapServer::allocateKernelMapCallback, this);
  save_map_srv_ = pnh_.advertiseService("save_act_map_layers",
                                        &ActMapServer::saveMapCallback, this);
  load_map_srv_ = pnh_.advertiseService("load_act_map_layers",
                                        &ActMapServer::loadMapCallback, this);

  // publish layers
  occ_layer_pub_ =
      pnh_.advertise<act_map_msgs::Layer>("occ_layer_out", 1, false);
  pos_factor_layer_pub_ =
      pnh_.advertise<act_map_msgs::Layer>("pos_factor_layer_out", 1, false);
  publish_map_srv_ = pnh_.advertiseService(
      "publish_act_map_layers", &ActMapServer::publishMapCallback, this);

  // subscribe layers
  occ_layer_sub_ = pnh_.subscribe("occ_layer_in", 1,
                                  &ActMapServer::occupancyLayerCallback, this);
  pos_factor_layer_sub_ = pnh_.subscribe(
      "pos_factor_layer_in", 1, &ActMapServer::kernelLayerCallback, this);
}

template <typename T>
void ActMapServer<T>::feature3dCallback(const PCLPointCloud::ConstPtr& pc,
                                        const size_t cam_idx)
{
  ros::Time pc_time;
  pcl_conversions::fromPCL(pc->header.stamp, pc_time);

  if (pc->empty())
  {
    visualizeAllUponPC(pc_time);
    return;
  }

  CHECK(pc->header.frame_id == kWorldFrame);  // TODO: directly publish in cam

  rpg::Pose T_w_c;

  tf::TransformListener listener;
  listener.waitForTransform(kWorldFrame, cam_frames_[cam_idx], pc_time,
                            ros::Duration(0.05));
  if (!rpg_ros::tf::get_T_A_B(kWorldFrame, cam_frames_[cam_idx], pc_time,
                              &T_w_c))
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
  visualizeAllUponPC(pc_time);
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
    VLOG(1) << "Recomputing kernel layer...";
    act_map_->recomputeFactorLayer();
    VLOG(1) << "Recomputed kernel layer. It took " << timer.stop() * 1000
            << " ms";
    act_map::utils::clearLayerUpdate(act_map_->occLayerPtr().get());
    pub_kvox_bestview_ = true;
  }
  else if (options_.ker_update_mode_ == KernelUpdateMode::kIncremental)
  {
    act_map_->updateFactorLayerIncremental();
    if (inc_step_cnt_ % options_.viz_bview_incremental_every_n_ == 0)
    {
      pub_kvox_bestview_ = true;
    }
    inc_step_cnt_++;
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
    act_map_->addRegionToFactorLayer(Twb, options_.kernel_expand_ranges_);
    VLOG(2) << "Expand kernel layer took " << timer.stop() * 1000 << " ms";
    pub_kernel_blk_centers_ = true;

    if (options_.only_activate_nearby_kernel_blks_)
    {
      const double block_half_dia_len = act_map_->posFactorBlockHalfDiagonal();
      act_map_->activateBlocksByDistance(Twb.getPosition(),
                                         options_.kernel_blks_activate_dist_ +
                                             block_half_dia_len);
    }
    prev_added_Twb = Twb;
  }
}

template <typename T>
void ActMapServer<T>::visualizeAllUponPC(const ros::Time& time)
{
  rpg::Pose T_offset;
  T_offset.setIdentity();
  publishCameraMarker(pub_markers_, cam_frames_[0], "cam", time, 1,
                      visualization_msgs::Marker::ADD,
                      options_.viz_cam_marker_size_, Eigen::Vector3d(0, 0, 1),
                      T_offset);

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
    act_map_->clearAccumulatedUpdatedFactorBlockIndices();
  }
}

template <typename T>
void ActMapServer<T>::updateVisualization() const
{
  visualizeOccupancy();
  visualizeKernelBlocks();
  visualizeKerBestViews(true);
}

template <typename T>
void ActMapServer<T>::visualizeOccupancy() const
{
  act_map::Vec3dVec occ_vox_cs;
  act_map_->getCentersOfOccupiedVoxels(&occ_vox_cs);
  VLOG(1) << "Going to publish " << occ_vox_cs.size() << " points.";
  PCLPointCloud occ_pc;
  Vec3dVecToPCLPointCloud(occ_vox_cs, &occ_pc);
  setPCLPointCloud(&occ_pc, kWorldFrame, ros::Time().now());
  occupied_vox_.publish(occ_pc);

  if (occ_view_dir_pub_.getNumSubscribers() > 0)
  {
    act_map::Vec3dVec occ_vox_dirs;
    act_map_->getViewDirsOfOccupiedVoxels(&occ_vox_dirs);
    CHECK_EQ(occ_vox_dirs.size(), occ_vox_cs.size());

    act_map::Vec3dVec valid_dirs_s, valid_dirs;
    for (size_t idx = 0; idx < occ_vox_dirs.size(); idx++)
    {
      if (std::fabs(occ_vox_dirs[idx].norm() - 1.0) > 1e-2)
      {
        continue;
      }
      valid_dirs_s.push_back(occ_vox_cs[idx]);
      valid_dirs.push_back(occ_vox_dirs[idx]);
    }
    // visualize
    std::vector<size_t> ids(valid_dirs_s.size());
    std::iota(ids.begin(), ids.end(), 0);
    std::vector<std_msgs::ColorRGBA> colors(valid_dirs_s.size());
    std_msgs::ColorRGBA c;
    c.r = 0.0;
    c.g = 0.0;
    c.b = 1.0;
    c.a = 1.0;
    std::fill(colors.begin(), colors.end(), c);
    visualization_msgs::MarkerArray ma;
    directionsToArrowsArray(valid_dirs_s, valid_dirs, ids, colors, "view_dir",
                            map_options_.occ_layer_options_.vox_size *
                                map_options_.occ_layer_options_.vox_per_side,
                            &ma);
    occ_view_dir_pub_.publish(ma);
  }
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
void ActMapServer<T>::visualizeKerBestViews(const bool viz_all) const
{
  act_map::Vec3dVec vox_cs;
  act_map::Vec3dVec bviews;

  rpg::Timer timer;
  timer.start();
  act_map::voxblox::LongIndexVector global_idxs;
  std::vector<double> values;

  const bool viz_updated = viz_all ? false : options_.viz_bview_last_updated_;

  act_map_->getBestViewsAt(0, options_.viz_bview_samples_per_side_,
                           options_.viz_bview_use_sampling_, viz_updated,
                           &vox_cs, &bviews, &values, &global_idxs);
  VLOG(1) << "Computing " << vox_cs.size() << " views for visulization took "
          << timer.stop() * 1000 << " ms.";

  visualization_msgs::MarkerArray ma;
  std::vector<size_t> ids;
  if (options_.viz_bview_use_unique_id_)
  {
    act_map::voxblox::LongIndexHash hasher;
    for (const act_map::voxblox::LongIndex& idx : global_idxs)
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

  double size = options_.viz_bview_size_;
  if (options_.viz_bview_size_vox_ratio_ > 0)
  {
    size = options_.viz_bview_size_vox_ratio_ *
           map_options_.pos_factor_layer_options_.vox_size;
  }
  directionsToArrowsArray(vox_cs, bviews, ids, colors, "bestview", size, &ma);
  kvox_bestviews_pub_.publish(ma);
}

template <typename T>
void ActMapServer<T>::saveMap(const std::string& path) const
{
  CHECK(rpg::fs::pathExists(path)) << path;
  const std::string occ_fn = path + "/occ_layer.protobuf";
  const std::string ker_fn =
      path + "/" + act_map::getVoxelType<T>() + "_layer.protobuf";
  act_map_->saveLayers(occ_fn, ker_fn);
}

template <typename T>
void ActMapServer<T>::loadMap(const std::string& path)
{
  CHECK(rpg::fs::pathExists(path));
  const std::string occ_fn = path + "/occ_layer.protobuf";
  const std::string ker_fn =
      path + "/" + act_map::getVoxelType<T>() + "_layer.protobuf";
  act_map_->loadLayers(occ_fn, ker_fn);
}

template <typename T>
void ActMapServer<T>::publishMap() const
{
  if (pos_factor_layer_pub_.getNumSubscribers() > 0)
  {
    rpg::Timer timer;
    timer.start();
    act_map_msgs::Layer msg;
    act_map::voxblox::serializeLayerAsMsg(act_map_->kerLayerCRef(), false,
                                          &msg);
    pos_factor_layer_pub_.publish(msg);
    VLOG(1) << "publishing kernel layer took (ms) " << 1000 * timer.stop();
  }

  if (occ_layer_pub_.getNumSubscribers() > 0)
  {
    rpg::Timer timer;
    timer.start();
    act_map_msgs::Layer msg;
    act_map::voxblox::serializeLayerAsMsg(act_map_->occLayerCRef(), false,
                                          &msg);
    occ_layer_pub_.publish(msg);
    VLOG(1) << "publishing occupancy layer took (ms) " << 1000 * timer.stop();
  }
}

template <typename T>
void ActMapServer<T>::recomputeMap()
{
  rpg::Timer timer;
  timer.start();
  act_map_->recomputeFactorLayer();
  VLOG(1) << "Recomputed kernel layer with "
          << act_map_->kerLayerPtr()->getNumberOfAllocatedBlocks() << " blocks "
          << "took " << timer.stop() * 1000 << " ms.";
}

template <typename T>
bool ActMapServer<T>::recomputeCallBack(std_srvs::Empty::Request& req,
                                        std_srvs::Empty::Response& res)
{
  recomputeMap();
  updateVisualization();

  return true;
}

template <typename T>
bool ActMapServer<T>::updateVisualizationCallback(
    std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  updateVisualization();
  return true;
}

template <typename T>
bool ActMapServer<T>::clearMapCallback(std_srvs::Empty::Request& req,
                                       std_srvs::Empty::Response& res)
{
  act_map_->clearMap();
  updateVisualization();
  return true;
}

template <typename T>
bool ActMapServer<T>::allocateKernelMapCallback(std_srvs::Empty::Request& req,
                                                std_srvs::Empty::Response& res)
{
  act_map_->allocateFactorLayerUniform(
      options_.preset_pos_factor_layer_ranges_);
  VLOG(1) << "Allocated "
          << act_map_->kerLayerPtr()->getNumberOfAllocatedBlocks()
          << " blocks in the kernel layer.";
  updateVisualization();
  return true;
}

template <typename T>
bool ActMapServer<T>::saveMapCallback(act_map_msgs::FilePath::Request& req,
                                      act_map_msgs::FilePath::Response& res)
{
  saveMap(req.file_path);
  return true;
}

template <typename T>
bool ActMapServer<T>::loadMapCallback(act_map_msgs::FilePath::Request& req,
                                      act_map_msgs::FilePath::Response& res)
{
  loadMap(req.file_path);
  return true;
}

template <typename T>
bool ActMapServer<T>::publishMapCallback(std_srvs::Empty::Request& req,
                                         std_srvs::Empty::Response& res)
{
  publishMap();
  return true;
}

template <typename T>
void ActMapServer<T>::kernelLayerCallback(const act_map_msgs::Layer& layer)
{
  bool success = act_map::voxblox::deserializeMsgToLayer(
      layer, act_map_->kerLayerPtr().get());
  if (!success)
  {
    LOG(WARNING) << "Fail to set the kernel layer.";
  }
  else
  {
    act_map_->setFactorLayerValid();
    act_map_->resetIncUpdateVars();
    VLOG(1) << "Loaded kernel layer from message.";
  }
  updateVisualization();
}

template <typename T>
void ActMapServer<T>::occupancyLayerCallback(const act_map_msgs::Layer& layer)
{
  bool success = act_map::voxblox::deserializeMsgToLayer(
      layer, act_map_->occLayerPtr().get());
  if (!success)
  {
    LOG(WARNING) << "Fail to set the occupancy layer.";
  }
  else
  {
    act_map_->setOccupancyLayerValid();
    act_map_->resetIncUpdateVars();
    VLOG(1) << "Loaded occupancy layer from message.";
  }
  updateVisualization();
}

template <typename T>
template <typename ST>
typename std::enable_if<!act_map::traits::is_vis_vox<ST>::value>::type
ActMapServer<T>::typedInit()
{
  LOG(WARNING) << "Going into dummy init function for this type.";
}

template <typename T>
template <typename ST>
typename std::enable_if<act_map::traits::is_gp_vis_vox<ST>::value>::type
ActMapServer<T>::typedInit()
{
  std::string vis_dir = rpg_ros::param(pnh_, "gp_vis_dir", std::string(""));
  if (vis_dir.empty())
  {
    LOG(WARNING) << "Cannot fid visibility approximator directory, return.";
    return;
  }
  ST::template setVisApproxFromFolderGP(vis_dir);
}

template <typename T>
template <typename ST>
typename std::enable_if<act_map::traits::is_quadpoly_vis_vox<ST>::value>::type
ActMapServer<T>::typedInit()
{
  ST::template setVisApproxQuadVisOpt(map_options_.vis_options_[0]);
}


}  // namespace act_map_ros
