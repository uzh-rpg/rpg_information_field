#pragma once

#include <glog/logging.h>
#include <memory>
#include <utility>

#include "act_map/voxblox/core/common.h"
#include "act_map/voxblox/core/layer.h"
#include "act_map/voxblox/core/voxel.h"

namespace act_map {

namespace voxblox {

class OccupancyMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<OccupancyMap> Ptr;

  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FloatingPoint occupancy_voxel_size = 0.2;
    size_t occupancy_voxels_per_side = 16u;
  };

  explicit OccupancyMap(const Config& config)
      : occupancy_layer_(new Layer<OccupancyVoxel>(
            config.occupancy_voxel_size, config.occupancy_voxels_per_side)) {
    block_size_ =
        config.occupancy_voxel_size * config.occupancy_voxels_per_side;
  }

  // Creates a new OccupancyMap based on a COPY of this layer.
  explicit OccupancyMap(const Layer<OccupancyVoxel>& layer)
      : OccupancyMap(aligned_shared<Layer<OccupancyVoxel>>(layer)) {}

  // Creates a new OccupancyMap that contains this layer.
  explicit OccupancyMap(Layer<OccupancyVoxel>::Ptr layer)
      : occupancy_layer_(layer) {
    CHECK(layer);
    block_size_ = layer->block_size();
  }

  virtual ~OccupancyMap() {}

  Layer<OccupancyVoxel>* getOccupancyLayerPtr() {
    return occupancy_layer_.get();
  }
  const Layer<OccupancyVoxel>& getOccupancyLayer() const {
    return *occupancy_layer_;
  }

  FloatingPoint block_size() const { return block_size_; }

 protected:
  FloatingPoint block_size_;

  // The layers.
  Layer<OccupancyVoxel>::Ptr occupancy_layer_;
};

}  // namespace voxblox

}  // namespace act_map

