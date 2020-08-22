#include "act_map_exp/exp_utils.h"

#include <random>
#include <rpg_common/test_main.h>

#include "act_map_exp/numdiff-jacobian-tester.h"

using namespace act_map_exp;

TEST(ExpUtilsTest, normalizeJac)
{
  struct VecNormFunctor : public aslam::common::NumDiffFunctor<3, 3>
  {
    VecNormFunctor(const double dummy)
    {
    }
    virtual ~VecNormFunctor()
    {
    }
    bool functional(const typename NumDiffFunctor<3, 3>::InputType& x,
                    typename NumDiffFunctor<3, 3>::ValueType& fvec,
                    typename NumDiffFunctor<3, 3>::JacobianType* Jout) const
    {
      fvec = x / x.norm();
      if (Jout)
      {
        jacVector3Normalization(x, Jout);
      }
      return true;
    }
  };

  double step_size = 1e-3;
  double test_tol = 1e-2;
  for (int i = 0; i < 50; i++)
  {
    Eigen::Vector3d v;
    v.setRandom();
    TEST_JACOBIAN_FINITE_DIFFERENCE(VecNormFunctor, v, step_size, test_tol, 0);
  }
}

TEST(ExpUtilsTest, skew)
{
  Eigen::Vector3d v;
  for (int i = 0; i < 50; i++)
  {
    v.setRandom();
    Eigen::Matrix3d m;
    skew(v, &m);
    EXPECT_TRUE(m.transpose().isApprox(-m));
  }
}

TEST(ExpUtilsTest, jacCrossProd)
{
  struct CrossProdFunctor : public aslam::common::NumDiffFunctor<3, 3>
  {
    CrossProdFunctor(const Eigen::Matrix3d& Ka, const Eigen::Matrix3d Kb)
      : Ka_(Ka), Kb_(Kb)
    {
    }
    virtual ~CrossProdFunctor()
    {
    }
    bool functional(const typename NumDiffFunctor<3, 3>::InputType& x,
                    typename NumDiffFunctor<3, 3>::ValueType& fvec,
                    typename NumDiffFunctor<3, 3>::JacobianType* Jout) const
    {
      Eigen::Vector3d a = Ka_ * x;
      Eigen::Vector3d b = Kb_ * x;
      fvec = a.cross(b);
      if (Jout)
      {
        jacVecCrossProd(a, b, Ka_, Kb_, Jout);
      }
      return true;
    }

    Eigen::Matrix3d Ka_, Kb_;
  };

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dim_dist(1, 10);

  double step_size = 1e-3;
  double test_tol = 1e-2;
  for (int i = 0; i < 50; i++)
  {
    const int rdim = dim_dist(gen);
    Eigen::VectorXd v(rdim);
    Eigen::Matrix3Xd Ka, Kb;
    Ka.resize(Eigen::NoChange, rdim);
    Kb.resize(Eigen::NoChange, rdim);
    v.setRandom();
    Ka.setRandom();
    Kb.setRandom();
    TEST_JACOBIAN_FINITE_DIFFERENCE(CrossProdFunctor, v, step_size, test_tol,
                                    Ka, Kb);
  }
}

TEST(ExpUtilsTest, recoverRwb)
{
  struct FlatToRwb : public aslam::common::NumDiffFunctor<9, 4>
  {
    FlatToRwb(const double dummy)
    {
    }
    virtual ~FlatToRwb()
    {
    }
    bool functional(const typename NumDiffFunctor<9, 4>::InputType& x,
                    typename NumDiffFunctor<9, 4>::ValueType& fvec,
                    typename NumDiffFunctor<9, 4>::JacobianType* Jout) const
    {
      const Eigen::Vector3d acc = x.block<3, 1>(0, 0);
      const double yaw_rad = x(3);
      rpg::Rotation::RotationMatrix rot_mat;
      if (Jout)
      {
        rpg::Matrix93 drotmat_dacc;
        rpg::Matrix91 drotmat_dyaw;
        quadAccYawToRwb(acc, yaw_rad, &rot_mat, &drotmat_dacc, &drotmat_dyaw);
        Jout->block<9, 3>(0, 0) = drotmat_dacc;
        Jout->block<9, 1>(0, 3) = drotmat_dyaw;
      }
      else
      {
        quadAccYawToRwb(acc, yaw_rad, &rot_mat, nullptr, nullptr);
      }

      fvec.block<3, 1>(0, 0) = rot_mat.row(0);
      fvec.block<3, 1>(3, 0) = rot_mat.row(1);
      fvec.block<3, 1>(6, 0) = rot_mat.row(2);
      return true;
    }
  };

  Eigen::Vector3d acc;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> yaw_dist(-M_PI, M_PI);
  double step_size = 1e-4;
  double test_tol = 1e-2;
  for (int i = 0; i < 50; i++)
  {
    acc.setRandom();
    Eigen::VectorXd v(4);
    v.head<3>() = acc;
    v(3) = yaw_dist(gen);
    TEST_JACOBIAN_FINITE_DIFFERENCE(FlatToRwb, v, step_size, test_tol, 0);
  }
}

TEST(ExpUtilsTest, matMultiplication)
{
  struct MatMul : public aslam::common::NumDiffFunctor<9, 9>
  {
    MatMul(const Eigen::Matrix3d& B) : B_(B)
    {
    }
    bool functional(const typename NumDiffFunctor::InputType& x,
                    typename NumDiffFunctor::ValueType& fvec,
                    typename NumDiffFunctor::JacobianType* Jout) const
    {
      Eigen::Matrix3d A;
      for (int i = 0; i < 9; i++)
      {
        A(i / 3, i % 3) = x[i];
      }

      Eigen::Matrix3d AB = A * B_;
      for (int i = 0; i < 9; i++)
      {
        fvec[i] = AB(i / 3, i % 3);
      }

      if (Jout)
      {
        squareMatMulJac(B_, Jout);
      }
      return true;
    }
    Eigen::Matrix3d B_;
  };

  double step_size = 1e-4;
  double test_tol = 1e-2;
  for (int i = 0; i < 50; i++)
  {
    MatMul::InputType x;
    Eigen::Matrix3d B;
    B.setRandom();
    x.setRandom();
    TEST_JACOBIAN_FINITE_DIFFERENCE(MatMul, x, step_size, test_tol, B);
  }
}

TEST(ExpUtilsTest, matVecMul)
{
  struct MatVecMul : public aslam::common::NumDiffFunctor<3, 9>
  {
    MatVecMul(const Eigen::Vector3d& v) : v_(v)
    {
    }
    bool functional(const typename NumDiffFunctor::InputType& x,
                    typename NumDiffFunctor::ValueType& fvec,
                    typename NumDiffFunctor::JacobianType* Jout) const
    {
      Eigen::Matrix3d A;
      for (int i = 0; i < 9; i++)
      {
        A(i / 3, i % 3) = x[i];
      }
      fvec = A * v_;
      if (Jout)
      {
        matVecMulJac(v_, Jout);
      }
      return true;
    }
    Eigen::Vector3d v_;
  };

  double step_size = 1e-4;
  double test_tol = 1e-2;
  for (int i = 0; i < 50; i++)
  {
    MatVecMul::InputType x;
    x.setRandom();
    Eigen::Vector3d v;
    v.setRandom();
    TEST_JACOBIAN_FINITE_DIFFERENCE(MatVecMul, x, step_size, test_tol, v);
  }
}

RPG_COMMON_TEST_MAIN
{
}
