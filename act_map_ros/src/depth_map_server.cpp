#include "act_map_ros/depth_map_server.h"

#include <rpg_common_ros/params_helper.h>
#include <rpg_common/fs.h>
#include <act_map/conversion.h>

#include "act_map_ros/params_reader.h"
#include "act_map_ros/common_ros.h"
#include "act_map_ros/conversion_ros.h"

namespace act_map_ros
{
DepthMapServer::DepthMapServer(const ros::NodeHandle& nh,
                               const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  readParams();
  init();
  setUpROS();
}

void DepthMapServer::readParams()
{
  options_.depth_layer_proto_fn_ =
      rpg_ros::param(pnh_, "depth_proto", std::string(""));
  CHECK(rpg::fs::fileExists(options_.depth_layer_proto_fn_));
  options_.feature_points_3d_fn_ =
      rpg_ros::param(pnh_, "points3d_txt", std::string(""));
  CHECK(rpg::fs::fileExists(options_.feature_points_3d_fn_));
  options_.max_depth_ =
      rpg_ros::param<double>(pnh_, "dm_server_max_depth", -1.0);

  options_.dm_options_ = readDepthMapOptions(pnh_);
}

void DepthMapServer::init()
{
  dm_ptr_ = std::make_shared<act_map::DepthMap>(options_.dm_options_);
  dm_ptr_->loadDepthLayer(options_.depth_layer_proto_fn_);
  this->resetVizStatus();

  // sanity check
  {
    act_map::DepthBlock::ConstPtr blk_ptr =
        dm_ptr_->getDepthBlockPtr(all_blk_idx_list_.front());
    CHECK_EQ(blk_ptr->voxels_per_side(),
             options_.dm_options_.depth_layer_opts_.vox_per_side);
    CHECK_EQ(blk_ptr->voxel_size(),
             options_.dm_options_.depth_layer_opts_.vox_size);
    CHECK_DOUBLE_EQ(blk_ptr->getVoxelByLinearIndex(0).getStepDeg(),
                    options_.dm_options_.depth_voxel_step_deg_);
  }

  VLOG(1) << "Load " << dm_ptr_->numAllocatedBlocks() << " blocks"
          << " with " << dm_ptr_->numAllocatedVoxels() << " voxels."
          << std::endl;

  points3d_ptr_ = std::make_shared<vi::Map>();
  points3d_ptr_->load(options_.feature_points_3d_fn_, std::string());
  VLOG(1) << "Load " << points3d_ptr_->points_.cols() << " 3D points.";
}

void DepthMapServer::resetVizStatus()
{
  dm_ptr_->depthLayerPtr()->getAllAllocatedBlocks(&all_blk_idx_list_);
  viz_vox_idx_ = 0;
  viz_blk_idx_ = 0;
}

void DepthMapServer::setUpROS()
{
  points3d_pub_ = pnh_.advertise<PCLPointCloud>("points3d", 5);
  block_centers_pub_ = pnh_.advertise<PCLPointCloud>("blk_centers", 10);
  sel_vox_centers_pub_ = pnh_.advertise<PCLPointCloud>("sel_vox_centers", 10);
  sel_vox_depths_pub_ = pnh_.advertise<PCLPointCloud>("sel_vox_depths", 10);
  markers_pub_ = pnh_.advertise<visualization_msgs::Marker>("viz_markers", 10);

  key_cmd_sub_ = pnh_.subscribe<std_msgs::String>(
      "/depth_map_cmd", 10, &DepthMapServer::keyCmdCallback, this);
}

void DepthMapServer::updateVisualization() const
{
  if (points3d_pub_.getNumSubscribers() > 0)
  {
    PCLPointCloud points3d_pc;
    act_map::Vec3dVec points3d_vec;
    act_map::eigenKXToVecKVec(points3d_ptr_->points_, &points3d_vec);
    VLOG(1) << "Publish " << points3d_vec.size() << " points.";
    Vec3dVecToPCLPointCloud(points3d_vec, &points3d_pc);
    setPCLPointCloud(&points3d_pc, kWorldFrame, ros::Time().now());
    points3d_pub_.publish(points3d_pc);
  }

  if (block_centers_pub_.getNumSubscribers() > 0)
  {
    PCLPointCloud blk_c_pc;
    act_map::Vec3dVec blk_c_vec;
    act_map::utils::getCentersOfAllBlocks(dm_ptr_->depthLayerCRef(),
                                          &blk_c_vec);
    VLOG(1) << "Publish " << blk_c_vec.size() << " block centers.";
    Vec3dVecToPCLPointCloud(blk_c_vec, &blk_c_pc);
    setPCLPointCloud(&blk_c_pc, kWorldFrame, ros::Time().now());
    block_centers_pub_.publish(blk_c_pc);
  }

  if (sel_vox_centers_pub_.getNumSubscribers() > 0)
  {
    PCLPointCloud vox_c_pc;
    act_map::Vec3dVec vox_c_vec;
    act_map::DepthBlock::ConstPtr active_blk =
        dm_ptr_->getDepthBlockPtr(all_blk_idx_list_[viz_blk_idx_]);
    act_map::utils::getCentersOfAllVoxels(*active_blk, &vox_c_vec);
    VLOG(1) << "Publish " << vox_c_vec.size() << " voxel centers.";
    Vec3dVecToPCLPointCloud(vox_c_vec, &vox_c_pc);
    setPCLPointCloud(&vox_c_pc, kWorldFrame, ros::Time().now());
    sel_vox_centers_pub_.publish(vox_c_pc);
  }

  visualizeSelectedVoxel();
}

void DepthMapServer::visualizeSelectedVoxel() const
{
  if (sel_vox_depths_pub_.getNumSubscribers() > 0)
  {
    act_map::DepthBlock::ConstPtr active_blk =
        dm_ptr_->getDepthBlockPtr(all_blk_idx_list_[viz_blk_idx_]);
    const act_map::DepthVoxel& vox =
        active_blk->getVoxelByLinearIndex(viz_vox_idx_);
    act_map::Vec3dVec points_w;
    vox.getDepthPoints(&points_w);
    PCLPointCloud points_w_pc;
    VLOG(1) << "Publish " << points_w.size() << " depth points.";
    Vec3dVecToPCLPointCloud(points_w, &points_w_pc);
    setPCLPointCloud(&points_w_pc, kWorldFrame, ros::Time().now());
    sel_vox_depths_pub_.publish(points_w_pc);
  }

  if (markers_pub_.getNumSubscribers() > 0)
  {
    act_map::DepthBlock::ConstPtr active_blk =
        dm_ptr_->getDepthBlockPtr(all_blk_idx_list_[viz_blk_idx_]);
    const act_map::DepthVoxel& vox =
        active_blk->getVoxelByLinearIndex(viz_vox_idx_);
    Eigen::Vector3d vox_c = vox.center();

    act_map::Vec3dVec points3d_vec;
    act_map::eigenKXToVecKVec(points3d_ptr_->points_, &points3d_vec);

    std::set<size_t> vis_idx;
    const double max_d =
        (options_.max_depth_ > 0 ? std::numeric_limits<double>::infinity() :
                                   options_.max_depth_);
    vox.getVisibleIdxFromPoints(points3d_vec, &vis_idx,
                                act_map::DepthVoxel::getMinMaxD<double>(0, max_d));

    visualization_msgs::Marker marker;
    marker.id = 0;
    marker.ns = "visibility";

    marker.header.frame_id = kWorldFrame;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 0.5;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;

    for (const size_t pt_i : vis_idx)
    {
      const Eigen::Vector3d& pw_i = points3d_vec[pt_i];
      geometry_msgs::Point point;
      point.x = static_cast<double>(vox_c(0));
      point.y = static_cast<double>(vox_c(1));
      point.z = static_cast<double>(vox_c(2));
      marker.points.push_back(point);
      point.x = static_cast<double>(pw_i(0));
      point.y = static_cast<double>(pw_i(1));
      point.z = static_cast<double>(pw_i(2));
      marker.points.push_back(point);
    }
    VLOG(1) << "Publish " << marker.points.size() / 2 << " visibility links.";
    markers_pub_.publish(marker);
  }
}

void DepthMapServer::keyCmdCallback(const std_msgs::StringConstPtr& key)
{
  const std::string& data = key->data;

  std::cout << "Received key " << data << std::endl;

  if (data == "r")
  {
    resetVizStatus();
  }
  else if (data == "b")
  {
    viz_blk_idx_ += 1;
    if (viz_blk_idx_ == all_blk_idx_list_.size())
    {
      viz_blk_idx_ = 0;
    }
  }
  else if (data == "v")
  {
    viz_vox_idx_ += 1;
    if (viz_vox_idx_ ==
        dm_ptr_->getDepthBlockPtr(all_blk_idx_list_[viz_blk_idx_])
            ->num_voxels())
    {
      viz_vox_idx_ = 0;
      viz_blk_idx_ += 1;
      if (viz_blk_idx_ == all_blk_idx_list_.size())
      {
        viz_blk_idx_ = 0;
      }
    }
  }

  updateVisualization();
}
}  // namespace act_map_ros
