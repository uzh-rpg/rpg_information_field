#include "act_map_exp/potential_cost.h"

#include <random>
#include <rpg_common/test_main.h>

#include "act_map_exp/numdiff-jacobian-tester.h"

using namespace act_map_exp;

TEST(PotentialCostTest, initAndEval)
{
  constexpr double robot_radius = 0.5;
  constexpr double dist_margin = 0.3;
  constexpr double cost_zero = 0.5 * dist_margin;
  PotentialCost p(robot_radius, dist_margin);

  double dist = -1.0;
  EXPECT_GT(p(dist), cost_zero);

  dist = 0.0;
  EXPECT_GT(p(dist), cost_zero);

  dist = robot_radius - 0.05;
  EXPECT_GT(p(dist), cost_zero);

  dist = robot_radius;
  EXPECT_DOUBLE_EQ(cost_zero, p(dist));

  dist = robot_radius + 0.5 * dist_margin;
  EXPECT_LT(p(dist), cost_zero);
  EXPECT_GT(p(dist), 0.0);

  dist = robot_radius + dist_margin;
  EXPECT_DOUBLE_EQ(0.0, p(dist));

  dist = robot_radius + dist_margin + 0.5;
  EXPECT_DOUBLE_EQ(0.0, p(dist));
}

TEST(PotentialCostTest, deriv)
{
  struct PotentialFunctor : public aslam::common::NumDiffFunctor<1, 1>
  {
    PotentialFunctor(PotentialCost* p) : p_(p)
    {
    }

    bool functional(const typename NumDiffFunctor::InputType& x,
                    typename NumDiffFunctor::ValueType& fvec,
                    typename NumDiffFunctor::JacobianType* Jout) const
    {
      if (Jout)
      {
        double jac;
        fvec(0, 0) = (*p_)(x(0, 0), &jac);
        (*Jout)(0, 0) = jac;
      }
      else
      {
        fvec(0, 0) = (*p_)(x(0, 0));
      }
      return true;
    }

    PotentialCost* p_;
  };

  constexpr double robot_radius = 0.5;
  constexpr double dist_margin = 0.3;
  constexpr double step_size = 1e-4;
  constexpr double test_tol = 1e-2;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist_d(-5.0, 5.0);
  PotentialCost p(robot_radius, dist_margin);
  for (int i = 0; i < 50; i++)
  {
    const double rd = dist_d(gen);
    Eigen::Matrix<double, 1, 1> input;
    input(0, 0) = rd;
    TEST_JACOBIAN_FINITE_DIFFERENCE(PotentialFunctor, input, step_size,
                                    test_tol, &p);
  }
}

RPG_COMMON_TEST_MAIN
{
}
