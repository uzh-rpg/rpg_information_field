#pragma once

#include <memory>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>

#include "act_map_exp/exp_utils.h"

namespace act_map_exp
{
using PolyParams = std::vector<Eigen::VectorXd>;
using RefConstXMat = Eigen::Ref<const Eigen::MatrixXd>;
template <int Row, int Col>
using RefConstMat = Eigen::Ref<const Eigen::Matrix<double, Row, Col>>;

template <int Order, int Dim>
class PolyTrajHelper
{
public:
  using Traj = mav_trajectory_generation::PolynomialOptimization<Order>;
  using DimMat = Eigen::Matrix<double, Eigen::Dynamic, Dim>;
  using DimVec = Eigen::Matrix<double, Dim, 1>;

  PolyTrajHelper() : poly_traj_(nullptr)
  {
  }
  PolyTrajHelper(const Traj* poly_traj_ptr) : poly_traj_(poly_traj_ptr)
  {
    init();
  }

  PolyTrajHelper(const PolyTrajHelper& rhs) = delete;

  inline void setPolyTraj(const Traj* traj_ptr)
  {
    poly_traj_ = traj_ptr;
    init();
  }

  void getAllDerivsAsMat(DimMat* derivs) const;

  void getCoefs(DimMat* coefs) const;

  inline void updateAllCoefs()
  {
    updateCoefs();
    updateVelCoefs();
    updateAccCoefs();
  }

  inline void updateCoefs()
  {
    getCoefs(&coefs_);
  }

  inline void updateVelCoefs()
  {
    multiplyCoefsPerSeg(coefs_, V_, &vel_coefs_);
  }

  inline void updateAccCoefs()
  {
    multiplyCoefsPerSeg(coefs_, A_, &acc_coefs_);
  }

  void multiplyCoefsPerSeg(const DimMat& coefs,
                           const Eigen::Matrix<double, Order, Order>& left_mat,
                           DimMat* out) const;

  inline const RefConstMat<Order, Dim> getCoefOfSeg(const size_t seg_i) const
  {
    return coefs_.template block<Order, Dim>(static_cast<int>(seg_i) * Order,
                                             0);
  }

  inline const RefConstMat<Order, Dim> getVelCoefOfSeg(const size_t seg_i) const
  {
    return vel_coefs_.template block<Order, Dim>(
        static_cast<int>(seg_i) * Order, 0);
  }

  inline const RefConstMat<Order, Dim> getAccCoefOfSeg(const size_t seg_i) const
  {
    return acc_coefs_.template block<Order, Dim>(
        static_cast<int>(seg_i) * Order, 0);
  }

  bool getSegmentIdxAndTime(const double t, size_t* seg_idx,
                            double* seg_t) const;

  void getTVec(const double t, Eigen::RowVectorXd* tvec) const
  {
    tvec->resize(Eigen::NoChange, Order);
    for (int ti = 0; ti < Order; ti++)
    {
      (*tvec)(ti) = std::pow(t, ti);
    }
  }

  void getPosition(const size_t seg_idx, const Eigen::RowVectorXd& tvec,
                   DimVec* pos, Eigen::RowVectorXd* dposi_ddp = nullptr) const;

  void getVelocity(const size_t seg_idx, const Eigen::RowVectorXd& tvec,
                   DimVec* vel, Eigen::RowVectorXd* dveli_ddp = nullptr) const;

  void getAcceleration(const size_t seg_idx, const Eigen::RowVectorXd& tvec,
                       DimVec* acc,
                       Eigen::RowVectorXd* dacci_ddp = nullptr) const;

  double getDynamicCost(PolyParams* grad) const;

  // fixed cache
  size_t n_fixed_;
  size_t n_free_;
  size_t n_seg_;

  Eigen::MatrixXd R_;
  Eigen::MatrixXd deriv_to_coef_;
  Eigen::MatrixXd dvelcoef_ddp_all_;
  Eigen::MatrixXd dacccoef_ddp_all_;
  // map position coef. to velocity coef.
  Eigen::Matrix<double, Order, Order> V_;
  // map position coef. to acceleration coef.
  Eigen::Matrix<double, Order, Order> A_;

  // need to be updated
  DimMat coefs_;
  DimMat vel_coefs_;
  DimMat acc_coefs_;

  const Traj* poly_traj_;

private:
  void init();
};

template <int Order, int Dim>
void PolyTrajHelper<Order, Dim>::init()
{
  CHECK_EQ(Dim, static_cast<int>(poly_traj_->getDimension()));
  Eigen::MatrixXd invA, M;
  n_free_ = poly_traj_->getNumberFreeConstraints();
  n_fixed_ = poly_traj_->getNumberFixedConstraints();
  n_seg_ = poly_traj_->getNumberSegments();
  poly_traj_->getR(&R_);
  poly_traj_->getAInverse(&invA);
  poly_traj_->getM(&M);
  deriv_to_coef_ = invA * M;

  V_.setZero();
  for (int i = 0; i < Order - 1; i++)
  {
    V_(i, i + 1) = i + 1;
  }
  dvelcoef_ddp_all_.resize(Order * static_cast<int>(n_seg_), n_free_);
  for (int i = 0; i < static_cast<int>(n_seg_); i++)
  {
    dvelcoef_ddp_all_.block(Order * i, 0, Order, n_free_) =
        V_ * deriv_to_coef_.block(Order * i, n_fixed_, Order, n_free_);
  }

  A_.setZero();
  for (int i = 0; i < Order - 2; i++)
  {
    A_(i, i + 2) = (i + 1) * (i + 2);
  }
  dacccoef_ddp_all_.resize(Order * static_cast<int>(n_seg_), n_free_);
  for (int i = 0; i < static_cast<int>(n_seg_); i++)
  {
    dacccoef_ddp_all_.block(Order * i, 0, Order, n_free_) =
        A_ * deriv_to_coef_.block(Order * i, n_fixed_, Order, n_free_);
  }
}

template <int Order, int Dim>
void PolyTrajHelper<Order, Dim>::getAllDerivsAsMat(DimMat* derivs) const
{
  PolyParams df, dp;
  poly_traj_->getFixedConstraints(&df);
  poly_traj_->getFreeConstraints(&dp);
  derivs->resize(n_fixed_ + n_free_, Eigen::NoChange);
  for (int i = 0; i < Dim; i++)
  {
    derivs->block(0, i, n_fixed_, 1) = df[i];
    derivs->block(n_fixed_, i, n_free_, 1) = dp[i];
  }
}

template <int Order, int Dim>
void PolyTrajHelper<Order, Dim>::getCoefs(DimMat* coefs) const
{
  DimMat derivs;
  getAllDerivsAsMat(&derivs);
  coefs->resize(Order * static_cast<int>(n_seg_), Eigen::NoChange);
  (*coefs) = deriv_to_coef_ * derivs;
}

template <int Order, int Dim>
void PolyTrajHelper<Order, Dim>::multiplyCoefsPerSeg(
    const DimMat& coefs, const Eigen::Matrix<double, Order, Order>& left_mat,
    DimMat* out) const
{
  CHECK_EQ(coefs.rows(), Order * static_cast<int>(n_seg_));

  out->resize(coefs.rows(), Eigen::NoChange);
  for (int i = 0; i < static_cast<int>(n_seg_); i++)
  {
    out->template block<Order, Dim>(i * Order, 0) =
        left_mat * coefs.template block<Order, Dim>(i * Order, 0);
  }
}

template <int Order, int Dim>
bool PolyTrajHelper<Order, Dim>::getSegmentIdxAndTime(const double t,
                                                      size_t* seg_idx,
                                                      double* seg_t) const
{
  CHECK_GT(t, 0.0);
  std::vector<double> seg_times;
  poly_traj_->getSegmentTimes(&seg_times);
  size_t i = 0;
  double acc_t = 0.0;
  for (i = 0; i < seg_times.size(); i++)
  {
    acc_t += seg_times[i];
    if (acc_t > t)
    {
      break;
    }
  }

  if (t > acc_t)
  {
    (*seg_idx) = 0;
    (*seg_t) = -1.0;
    return false;
  }

  (*seg_idx) = i;
  (*seg_t) = t - (acc_t - seg_times[i]);
  return true;
}

template <int Order, int Dim>
void PolyTrajHelper<Order, Dim>::getPosition(
    const size_t seg_idx, const Eigen::RowVectorXd& tvec, DimVec* pos,
    Eigen::RowVectorXd* dposi_ddp) const
{
  (*pos) = (tvec * getCoefOfSeg(seg_idx)).transpose();
  if (dposi_ddp)
  {
    (*dposi_ddp) = tvec * deriv_to_coef_.template block(
                              seg_idx * Order, n_fixed_, Order, n_free_);
  }
}

template <int Order, int Dim>
void PolyTrajHelper<Order, Dim>::getVelocity(
    const size_t seg_idx, const Eigen::RowVectorXd& tvec, DimVec* vel,
    Eigen::RowVectorXd* dveli_ddp) const
{
  (*vel) = (tvec * getVelCoefOfSeg(seg_idx)).transpose();
  if (dveli_ddp)
  {
    (*dveli_ddp) =
        tvec * dvelcoef_ddp_all_.block(Order * seg_idx, 0, Order, n_free_);
  }
}

template <int Order, int Dim>
void PolyTrajHelper<Order, Dim>::getAcceleration(
    const size_t seg_idx, const Eigen::RowVectorXd& tvec, DimVec* acc,
    Eigen::RowVectorXd* dacci_ddp) const
{
  (*acc) = (tvec * getAccCoefOfSeg(seg_idx)).transpose();
  if (dacci_ddp)
  {
    (*dacci_ddp) =
        tvec * dacccoef_ddp_all_.block(Order * seg_idx, 0, Order, n_free_);
  }
}

template <int Order, int Dim>
double PolyTrajHelper<Order, Dim>::getDynamicCost(PolyParams* grad) const
{
  double cost = 0.0;

  DimMat derivs;
  getAllDerivsAsMat(&derivs);
  Eigen::Matrix<double, Dim, Eigen::Dynamic> mid = derivs.transpose() * R_;
  for (int i = 0; i < Dim; i++)
  {
    cost += mid.row(i) * derivs.col(i);
  }


  if (grad)
  {
    CHECK_EQ(Dim, static_cast<int>(grad->size()));
    CHECK_EQ(n_free_, static_cast<size_t>(grad->front().size()));
    setZero(grad);
    const Eigen::Ref<const Eigen::MatrixXd> R_pf =
        R_.block(n_fixed_, 0, n_free_, n_fixed_);
    const Eigen::Ref<const Eigen::MatrixXd> R_pp =
        R_.block(n_fixed_, n_fixed_, n_free_, n_free_);

    for (size_t i = 0; i < Dim; i++)
    {
      (*grad)[i] = 2 * R_pf * derivs.block(0, i, n_fixed_, 1) +
                   2 * R_pp.transpose() * derivs.block(n_fixed_, i, n_free_, 1);
    }
  }

  return cost;
}

}  // namespace act_map_exp
