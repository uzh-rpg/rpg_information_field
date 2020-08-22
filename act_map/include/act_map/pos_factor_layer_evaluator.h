#pragma once

#include "act_map/positional_factor_voxel_ops.h"
#include "act_map/info_utils.h"
#include "act_map/interpolation.h"

#include <rpg_common/pose.h>

namespace act_map
{
enum class EvaluateStrategy
{
  kNearestNeighbour,
  kInterpolation
};

template <typename T, typename EvalT>
class PositionalFactorLayerEvaluatorBase
{
public:
  using GradT = Eigen::Vector3d;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PositionalFactorLayerEvaluatorBase()
  {
  }
  ~PositionalFactorLayerEvaluatorBase()
  {
  }

  explicit PositionalFactorLayerEvaluatorBase(
      const voxblox::Layer<T>* pos_factor_layer_ptr)
    : factor_layer_ptr_(pos_factor_layer_ptr)
  {
    numeric_pos_step_ = 0.1 * factor_layer_ptr_->voxel_size();
  }

  void setKernelLayer(const voxblox::Layer<T>* pos_factor_layer_ptr)
  {
    factor_layer_ptr_ = pos_factor_layer_ptr;
    numeric_pos_step_ = 0.1 * factor_layer_ptr_->voxel_size();
  }

  void setQuadraticCoefs(const double k1, const double k2, const double k3)
  {
    k1_ = k1;
    k2_ = k2;
    k3_ = k3;
  }

  void setNumericalStepRatio(const double ratio_of_vox_size)
  {
    numeric_pos_step_ = ratio_of_vox_size * factor_layer_ptr_->voxel_size();
  }

  const voxblox::Layer<T>* layerPtr() const
  {
    return factor_layer_ptr_;
  }

  const T* getVoxelNearest(const Eigen::Vector3d& pt) const
  {
    CHECK(factor_layer_ptr_) << "No layer set yet.";
    voxblox::BlockIndex bidx =
        factor_layer_ptr_->computeBlockIndexFromCoordinates(pt);
    typename voxblox::Block<T>::ConstPtr bptr =
        factor_layer_ptr_->getBlockPtrByIndex(bidx);
    if (!bptr)
    {
      return nullptr;
    }
    return bptr->getVoxelPtrByCoordinates(pt);
  }

  bool getValueNearest(const rpg::Pose& Twc, const InfoMetricType& v,
                       double* val, GradT* dpos = nullptr,
                       GradT* drot_g = nullptr) const
  {
    const double vox_size = factor_layer_ptr_->voxel_size();
    const double vox_size_x2 = 2 * vox_size;

    const T* vox = getVoxelNearest(Twc.getPosition());
    if (vox == nullptr)
    {
      return false;
    }

    (*val) = impl()->getValueFromVoxel(Twc.getRotationMatrix(), vox, v, drot_g);

    if (dpos)
    {
      for (int i = 0; i < 3; i++)
      {
        rpg::Pose Twc_m = Twc;
        rpg::Pose Twc_p = Twc;

        double val_m = 0.0;
        double val_p = 0.0;
        bool res = true;
        (Twc_m.getPosition()(i)) -= vox_size;
        (Twc_p.getPosition()(i)) += vox_size;
        res &= getValueNearest(Twc_m, v, &val_m, nullptr, nullptr);
        res &= getValueNearest(Twc_p, v, &val_p, nullptr, nullptr);
        if (res == false)
        {
          //          LOG(WARNING) << "Getting nearby values failed for
          //          dimension " << i;
          return false;
        }

        (*dpos)(i) = (val_p - val_m) / vox_size_x2;
      }
    }

    return true;
  }

  bool getValueInterpolation(const rpg::Pose& Twc, const InfoMetricType& v,
                             double* val, GradT* dpos = nullptr,
                             GradT* drot_g = nullptr) const
  {
    // get surrounding voxels
    InterpVoxels<T> sur_voxs;
    InterpVoxCenters sur_vox_cs;
    QueryVoxelsRes res = getSurroundVoxels(
        *factor_layer_ptr_, Twc.getPosition(), &sur_voxs, &sur_vox_cs);
    if (res != QueryVoxelsRes::kNearest && res != QueryVoxelsRes::kSurround)
    {
      LOG(WARNING) << "Failed to get voxels at "
                   << Twc.getPosition().transpose() << " with the result "
                   << static_cast<std::underlying_type<QueryVoxelsRes>::type>(
                          res);
      return false;
    }

    if (res == QueryVoxelsRes::kNearest)
    {
      const T* vox = sur_voxs.at(0);
      (*val) =
          impl()->getValueFromVoxel(Twc.getRotationMatrix(), vox, v, drot_g);
    }
    else if (res == QueryVoxelsRes::kSurround)
    {
      (*val) = impl()->getValueFromVoxelSurround(
          Twc, sur_voxs, sur_vox_cs, factor_layer_ptr_->voxel_size(), v,
          drot_g);
    }

    if (dpos)
    {
      for (int i = 0; i < 3; i++)
      {
        rpg::Pose Twc_m = Twc;
        rpg::Pose Twc_p = Twc;
        (Twc_m.getPosition()(i)) -= numeric_pos_step_;
        (Twc_p.getPosition()(i)) += numeric_pos_step_;

        bool res = true;
        double val_m = 0.0;
        double val_p = 0.0;
        res &= getValueInterpolation(Twc_m, v, &val_m, nullptr, nullptr);
        res &= getValueInterpolation(Twc_p, v, &val_p, nullptr, nullptr);
        if (res == false)
        {
          //          LOG(WARNING) << "Getting nearby values failed for
          //          dimension " << i;
          return false;
        }
        (*dpos)(i) = (val_p - val_m) / (2 * numeric_pos_step_);
      }
    }

    return true;
  }

protected:
  const EvalT* impl() const
  {
    return static_cast<const EvalT*>(this);
  }
  const voxblox::Layer<T>* factor_layer_ptr_;
  double numeric_pos_step_;
  // for quadratic kernels
  double k1_ = 0;
  double k2_ = 0;
  double k3_ = 0;
};

// to enable selection of types: specialization for multiple types
template <typename T, typename VoidT = void>
class PosFactorLayerEvaluator
{
public:
};

// visibility kernel voxel
template <typename T>
class PosFactorLayerEvaluator<
    T, typename std::enable_if<traits::is_vis_vox<T>::value>::type>
    : public PositionalFactorLayerEvaluatorBase<
          T,
          PosFactorLayerEvaluator<
              T, typename std::enable_if<traits::is_vis_vox<T>::value>::type>>
{
public:
  using PositionalFactorLayerEvaluatorBase<
      T, PosFactorLayerEvaluator<
             T, typename std::enable_if<traits::is_vis_vox<T>::value>::type>>::
      PositionalFactorLayerEvaluatorBase;
  using Ptr = std::shared_ptr<PosFactorLayerEvaluator>;
  double getValueFromVoxel(const Eigen::Matrix3d& Rwc, const T* vox,
                           const InfoMetricType& v,
                           Eigen::Vector3d* drot_global) const
  {
    return getInfoMetricAtRotationFromPositionalFactor<T>(
        Rwc, vox->getFactorConstRef(), v, drot_global);
  }
  double getValueFromVoxelSurround(const rpg::Pose& Twc,
                                   const InterpVoxels<T>& sur_voxs,
                                   const InterpVoxCenters& sur_vox_cs,
                                   const double vox_size,
                                   const InfoMetricType& v,
                                   Eigen::Vector3d* drot_global) const
  {
    const Eigen::Vector3d& pos = Twc.getPosition();
    const Eigen::Matrix3d& Rwc = Twc.getRotationMatrix();

    InterpValues<typename T::FactorType> values;
    for (size_t i = 0; i < 8; i++)
    {
      values.at(i) = &(sur_voxs.at(i)->getFactorConstRef());
    }

    typename T::FactorType interped_ker;
    trilinearInterpolation(pos, sur_vox_cs.col(0), vox_size, values,
                           &interped_ker);
    double val = getInfoMetricAtRotationFromPositionalFactor<T>(
        Rwc, interped_ker, v, drot_global);

    return val;
  }
};

// quadratic kernel voxel
template <typename T>
class PosFactorLayerEvaluator<
    T, typename std::enable_if<traits::is_quad_vox<T>::value>::type>
    : public PositionalFactorLayerEvaluatorBase<
          T,
          PosFactorLayerEvaluator<
              T, typename std::enable_if<traits::is_quad_vox<T>::value>::type>>
{
public:
  using PositionalFactorLayerEvaluatorBase<
      T, PosFactorLayerEvaluator<
             T, typename std::enable_if<traits::is_quad_vox<T>::value>::type>>::
      PositionalFactorLayerEvaluatorBase;
  using Ptr = std::shared_ptr<PosFactorLayerEvaluator>;
  double getValueFromVoxel(const Eigen::Matrix3d& Rwc, const T* vox,
                           const InfoMetricType& v,
                           Eigen::Vector3d* drot_global) const
  {
    return getInfoMetricAtRotationFromPositionalFactor(
        Rwc, this->k1_, this->k2_, this->k3_, vox->K1, vox->K2, vox->K3, v,
        drot_global);
  }
  double getValueFromVoxelSurround(const rpg::Pose& Twc,
                                   const InterpVoxels<T>& sur_voxs,
                                   const InterpVoxCenters& sur_vox_cs,
                                   const double vox_size,
                                   const InfoMetricType& v,
                                   Eigen::Vector3d* drot_global) const
  {
    const Eigen::Vector3d& pos = Twc.getPosition();
    const Eigen::Matrix3d& Rwc = Twc.getRotationMatrix();

    InterpValues<typename T::K1Type> K1_values;
    InterpValues<typename T::K2Type> K2_values;
    InterpValues<typename T::K3Type> K3_values;
    for (size_t i = 0; i < 8; i++)
    {
      K1_values.at(i) = &(sur_voxs.at(i)->K1);
      K2_values.at(i) = &(sur_voxs.at(i)->K2);
      K3_values.at(i) = &(sur_voxs.at(i)->K3);
    }

    typename T::K1Type K1_interped;
    typename T::K2Type K2_interped;
    typename T::K3Type K3_interped;
    trilinearInterpolation(pos, sur_vox_cs.col(0), vox_size, K1_values,
                           &K1_interped);

    trilinearInterpolation(pos, sur_vox_cs.col(0), vox_size, K2_values,
                           &K2_interped);

    trilinearInterpolation(pos, sur_vox_cs.col(0), vox_size, K3_values,
                           &K3_interped);

    double val = getInfoMetricAtRotationFromPositionalFactor(
        Rwc, this->k1_, this->k2_, this->k3_, K1_interped, K2_interped,
        K3_interped, v, drot_global);

    return val;
  }
};  // LayerEvaluator class
}  // namespace act_map
