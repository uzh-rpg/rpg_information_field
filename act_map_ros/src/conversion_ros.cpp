#include <act_map_ros/conversion_ros.h>

#include <glog/logging.h>

namespace act_map_ros
{
void Vec3dVecToPCLPointCloud(
    const act_map::Vec3dVec& vec, PCLPointCloud* pcl_pc)
{
  // as unordered pointcloud
  pcl_pc->height = 1;
  pcl_pc->width = static_cast<int>(vec.size());
  for (size_t i = 0; i < vec.size(); i++)
  {
    pcl_pc->push_back(
        pcl::PointXYZ(vec[i].x(), vec[i].y(), vec[i].z()));
  }
}
}
