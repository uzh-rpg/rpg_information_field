#include "act_map_exp/poly_traj_helper.h"

#include <random>

#include <rpg_common/test_main.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include "act_map_exp/numdiff-jacobian-tester.h"

using namespace act_map_exp;
namespace mtg = mav_trajectory_generation;

template <int Order, int Dim, int NFree>
struct TrajDerivFunctor
  : public aslam::common::NumDiffFunctor<Dim * 3 + 1, NFree * Dim>
{
  using NumDiffT = aslam::common::NumDiffFunctor<Dim * 3 + 1, NFree * Dim>;
  TrajDerivFunctor(PolyTrajHelper<Order, Dim>* pth,
                   mtg::PolynomialOptimization<Order>* traj,
                   const double sample_t)
    : pth_(pth), traj_(traj), sample_t_(sample_t)
  {
  }

  bool functional(const typename NumDiffT::InputType& x,
                  typename NumDiffT::ValueType& fvec,
                  typename NumDiffT::JacobianType* Jout) const
  {
    PolyParams free_constraints(Dim);
    for (int i = 0; i < Dim; i++)
    {
      free_constraints[static_cast<size_t>(i)] =
          x.template block<NFree, 1>(i * NFree, 0);
    }
    traj_->setFreeConstraints(free_constraints);
    pth_->updateCoefs();
    pth_->updateVelCoefs();
    pth_->updateAccCoefs();
    size_t seg_idx;
    double seg_t;
    CHECK(pth_->getSegmentIdxAndTime(sample_t_, &seg_idx, &seg_t));
    Eigen::RowVectorXd t_vec;
    pth_->getTVec(sample_t_, &t_vec);

    Eigen::Matrix<double, Dim, 1> pos, vel, acc;
    double dyn_cost;
    if (Jout)
    {
      Eigen::RowVectorXd dposi_ddp, dveli_dpp, dacci_ddp;
      pth_->getPosition(seg_idx, t_vec, &pos, &dposi_ddp);
      pth_->getVelocity(seg_idx, t_vec, &vel, &dveli_dpp);
      pth_->getAcceleration(seg_idx, t_vec, &acc, &dacci_ddp);
      PolyParams grad(Dim, Eigen::VectorXd(NFree));
      dyn_cost = pth_->getDynamicCost(&grad);
      Jout->setZero();
      for (int i = 0; i < Dim; i++)
      {
        Jout->template block<1, NFree>(i, i * NFree) = dposi_ddp;
      }
      for (int i = 0; i < Dim; i++)
      {
        Jout->template block<1, NFree>(i + Dim, i * NFree) = dveli_dpp;
      }
      for (int i = 0; i < Dim; i++)
      {
        Jout->template block<1, NFree>(i + Dim * 2, i * NFree) = dacci_ddp;
      }
      for (int i = 0; i < Dim; i++)
      {
        Jout->template block<1, NFree>(Dim * 3, i * NFree) =
            grad[static_cast<size_t>(i)].transpose();
      }
    }
    else
    {
      pth_->getPosition(seg_idx, t_vec, &pos, nullptr);
      pth_->getVelocity(seg_idx, t_vec, &vel, nullptr);
      pth_->getAcceleration(seg_idx, t_vec, &acc, nullptr);
      dyn_cost = pth_->getDynamicCost(nullptr);
    }
    fvec.template block<Dim, 1>(0, 0) = pos;
    fvec.template block<Dim, 1>(Dim, 0) = vel;
    fvec.template block<Dim, 1>(Dim * 2, 0) = acc;
    fvec(Dim * 3, 0) = dyn_cost;

    return true;
  }

  PolyTrajHelper<Order, Dim>* pth_;
  mtg::PolynomialOptimization<Order>* traj_;
  const double sample_t_;
};

class PolyTrajHelperTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<double> time_dist(10.0, 20.0);
    std::uniform_real_distribution<double> start_dist(-20.0, -10.0);
    std::uniform_real_distribution<double> end_dist(10.0, 20.0);

    std::uniform_real_distribution<double> yaw_dist(-M_PI, M_PI);

    total_time_ = time_dist(gen);
    const std::vector<double> seg_times(kNSeg, total_time_ / kNSeg);

    // position
    pos_poly_opt_.reset(new mtg::PolynomialOptimization<10>(3));
    {
      Eigen::Vector3d start, end;
      start.setRandom();
      start *= start_dist(gen);
      end.setRandom();
      end *= end_dist(gen);
      mtg::Vertex::Vector pos_vertices(kNSeg + 1, mtg::Vertex(3));
      pos_vertices.front().makeStartOrEnd(start, kPosOptDeriv);
      pos_vertices.back().makeStartOrEnd(end, kPosOptDeriv);
      pos_poly_opt_->setupFromVertices(pos_vertices, seg_times, kPosOptDeriv);
    }

    yaw_poly_opt_.reset(new mtg::PolynomialOptimization<6>(1));
    {
      mtg::Vertex::Vector yaw_vertices(kNSeg + 1, mtg::Vertex(1));
      yaw_vertices.front().makeStartOrEnd(yaw_dist(gen), kYawOptDeriv);
      yaw_vertices.back().makeStartOrEnd(yaw_dist(gen), kYawOptDeriv);
      yaw_poly_opt_->setupFromVertices(yaw_vertices, seg_times, kYawOptDeriv);
    }
    pos_traj_helper_.setPolyTraj(pos_poly_opt_.get());
    yaw_traj_helper_.setPolyTraj(yaw_poly_opt_.get());
    CHECK_EQ(pos_poly_opt_->getNumberFreeConstraints(), kNFreePos);
    CHECK_EQ(yaw_poly_opt_->getNumberFreeConstraints(), kNFreeYaw);
  }

  static constexpr size_t kNSeg = 3;
  static constexpr int kPosOptDeriv = mtg::derivative_order::SNAP;
  static constexpr int kYawOptDeriv = mtg::derivative_order::ACCELERATION;
  static constexpr size_t kNFreePos = (kNSeg - 1) * (kPosOptDeriv + 1);
  static constexpr size_t kNFreeYaw = (kNSeg - 1) * (kYawOptDeriv + 1);
  double total_time_;
  std::shared_ptr<mtg::PolynomialOptimization<10>> pos_poly_opt_;
  std::shared_ptr<mtg::PolynomialOptimization<6>> yaw_poly_opt_;

  PolyTrajHelper<10, 3> pos_traj_helper_;
  PolyTrajHelper<6, 1> yaw_traj_helper_;

  using PosDerivFunctor = TrajDerivFunctor<10, 3, kNFreePos>;
  using YawDerivFuctor = TrajDerivFunctor<6, 1, kNFreeYaw>;
};

TEST_F(PolyTrajHelperTest, derivatives)
{
  constexpr double step_size = 1e-4;
  constexpr double test_tol = 1e-2;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> time_sampler(0.01, total_time_ - 0.01);
  for (int i = 0; i < 50; i++)
  {
    const double sample_t = time_sampler(gen);
    Eigen::Matrix<double, 3 * kNFreePos, 1> input_pos;
    input_pos.setRandom();
    TEST_JACOBIAN_FINITE_DIFFERENCE(PosDerivFunctor, input_pos, step_size,
                                    test_tol, &pos_traj_helper_,
                                    pos_poly_opt_.get(), sample_t);
    Eigen::Matrix<double, kNFreeYaw, 1> input_yaw;
    input_yaw.setRandom();
    TEST_JACOBIAN_FINITE_DIFFERENCE(YawDerivFuctor, input_yaw, step_size,
                                    test_tol, &yaw_traj_helper_,
                                    yaw_poly_opt_.get(), sample_t);
  }
}

TEST_F(PolyTrajHelperTest, values)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> time_sampler(0.01, total_time_ - 0.01);

  PolyParams pos_free_constraints(3, Eigen::VectorXd(kNFreePos));
  setRandom(&pos_free_constraints);
  PolyParams yaw_free_constraints(1, Eigen::VectorXd(kNFreeYaw));
  setRandom(&yaw_free_constraints);
  pos_poly_opt_->setFreeConstraints(pos_free_constraints);
  yaw_poly_opt_->setFreeConstraints(yaw_free_constraints);
  mtg::Trajectory pos_traj, yaw_traj;
  pos_poly_opt_->getTrajectory(&pos_traj);
  yaw_poly_opt_->getTrajectory(&yaw_traj);
  pos_traj_helper_.updateAllCoefs();
  yaw_traj_helper_.updateAllCoefs();

  for (int i = 0; i < 50; i++)
  {
    const double sample_t = time_sampler(gen);

    {
      size_t seg_idx;
      double seg_t;
      pos_traj_helper_.getSegmentIdxAndTime(sample_t, &seg_idx, &seg_t);
      Eigen::RowVectorXd t_vec;
      pos_traj_helper_.getTVec(seg_t, &t_vec);
      mav_msgs::EigenTrajectoryPoint state;
      mtg::sampleTrajectoryAtTime(pos_traj, sample_t, &state);
      Eigen::Vector3d pos, vel, acc;
      pos_traj_helper_.getPosition(seg_idx, t_vec, &pos, nullptr);
      pos_traj_helper_.getVelocity(seg_idx, t_vec, &vel, nullptr);
      pos_traj_helper_.getAcceleration(seg_idx, t_vec, &acc, nullptr);
      EXPECT_TRUE(pos.isApprox(state.position_W, 1e-4))
          << pos.transpose() << " vs " << state.position_W.transpose();
      EXPECT_TRUE(vel.isApprox(state.velocity_W, 1e-4))
          << vel.transpose() << " vs " << state.velocity_W.transpose();
      EXPECT_TRUE(acc.isApprox(state.acceleration_W, 1e-4))
          << acc.transpose() << " vs " << state.acceleration_W.transpose();

      const double dyn_cost = pos_traj_helper_.getDynamicCost(nullptr);
      EXPECT_NEAR(2 * pos_poly_opt_->computeCost(), dyn_cost, 1e-6);
    }
    {
      double yaw;
      Eigen::VectorXd yaw_vec =
          yaw_traj.evaluate(sample_t, mtg::derivative_order::POSITION);
      yaw = yaw_vec(0);

      size_t seg_idx;
      double seg_t;
      yaw_traj_helper_.getSegmentIdxAndTime(sample_t, &seg_idx, &seg_t);
      Eigen::RowVectorXd t_vec;
      yaw_traj_helper_.getTVec(seg_t, &t_vec);
      Eigen::Matrix<double, 1, 1> yaw_mat;
      yaw_traj_helper_.getPosition(seg_idx, t_vec, &yaw_mat, nullptr);
      EXPECT_NEAR(yaw, yaw_mat(0, 0), 1e-4);
      const double dyn_cost = yaw_traj_helper_.getDynamicCost(nullptr);
      EXPECT_NEAR(2 * yaw_poly_opt_->computeCost(), dyn_cost, 1e-6);
    }
  }
}

RPG_COMMON_TEST_MAIN
{
}
