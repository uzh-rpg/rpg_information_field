//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include "act_map/kernel_voxel_ops.h"

namespace act_map
{
enum class EvaluateStrategy
{
  kNearestNeighbour,
  kInterpolation
};

template <typename T>
class LayerEvaluator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LayerEvaluator()
  {
  }
  ~LayerEvaluator()
  {
  }

  LayerEvaluator(const voxblox::Layer<T>* ker_layer_ptr)
    : ker_layer_ptr_(ker_layer_ptr)
  {
  }

  void setKernelLayer(const voxblox::Layer<T>* ker_layer_ptr)
  {
    ker_layer_ptr_ = ker_layer_ptr;
  }

  const voxblox::Layer<T>* layerPtr() const
  {
    return ker_layer_ptr_;
  }

  const T* getVoxelAt(
      const Eigen::Vector3d& pt,
      const EvaluateStrategy s = EvaluateStrategy::kNearestNeighbour) const
  {
    if (s == EvaluateStrategy::kNearestNeighbour)
    {
      return getVoxelNearest(pt);
    }
    else if (s == EvaluateStrategy::kInterpolation)
    {
      LOG(FATAL) << "Not implemented yet.";
    }
  }

  const T* getVoxelNearest(const Eigen::Vector3d& pt) const
  {
    CHECK(ker_layer_ptr_) << "No layer set yet.";
    voxblox::BlockIndex bidx =
        ker_layer_ptr_->computeBlockIndexFromCoordinates(pt);
    typename voxblox::Block<T>::ConstPtr bptr =
        ker_layer_ptr_->getBlockPtrByIndex(bidx);
    if (!bptr)
    {
      return nullptr;
    }
    return bptr->getVoxelPtrByCoordinates(pt);
  }

private:
  const voxblox::Layer<T>* ker_layer_ptr_;
};
}
