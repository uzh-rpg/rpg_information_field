#include "act_map/information_potential.h"

#include <random>

#include <rpg_common/test_main.h>
#include <rpg_common/save.h>
#include <rpg_common/fs.h>
#include <vi_utils/numdiff-jacobian-tester.h>
#include <vi_utils/common_utils.h>

#include "act_map/positional_factor_voxel.h"

using namespace act_map;

template <typename T>
class InfoPotentialTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::string dir, fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    std::string vis_dir = dir + "/../params/fov_approximator_gp/"
                                "fov40_fs50_lm1000_k10_fast";
    CHECK(rpg::fs::pathExists(vis_dir));
    setGPVisiblityFromFolder(vis_dir);
    setQuadPolyVisiblity(QuadVisScoreOptions());
  }
};

using KernelVoxelTypes =
    ::testing::Types<QuadInfoVoxel, QuadTraceVoxel, GPInfoVoxel, GPTraceVoxel,
                     QuadPolyInfoVoxel, QuadPolyTraceVoxel>;
TYPED_TEST_CASE(InfoPotentialTest, KernelVoxelTypes);

TYPED_TEST(InfoPotentialTest, init)
{
  kInfoMetricUseLogDet = true;
  InfoPotentialOptions options;
  options.average_over_n_ = 1;
  options.max_depth_m_ = 1.0;
  options.min_depth_m_ = 0.3;
  options.n_random_landmarks_ = 5.0;
  InformationPotential<TypeParam> info_pot(options);
  EXPECT_TRUE(info_pot.isInitialized());
}

TYPED_TEST(InfoPotentialTest, initInfo)
{
  InfoPotentialOptions options;
  InformationPotential<TypeParam> info_pot(options);

  std::cout << "Information Metric for factor type "
            << getVoxelType<TypeParam>() << std::endl;
  for (const InfoMetricType& info_t :
       InformationPotential<TypeParam>::kValidMetric)
  {
    const double info_val = info_pot.getMetricThresh(info_t);
    const double approx_info_val = info_pot.getMetricThreshApproxVis(info_t);
    EXPECT_GT(info_val, 0.0);
    std::cout << "- normal " << kInfoMetricNames.at(info_t) << ": " << info_val
              << std::endl;
    std::cout << "- approx: " << kInfoMetricNames.at(info_t) << ": "
              << approx_info_val << std::endl;
  }
}

TYPED_TEST(InfoPotentialTest, relative)
{
  {
    InfoPotentialOptions near_opts;
    near_opts.min_depth_m_ = 0.5;
    near_opts.max_depth_m_ = 1.0;
    InformationPotential<TypeParam> info_pot_near(near_opts);

    InfoPotentialOptions far_opts;
    far_opts.min_depth_m_ = 0.8;
    far_opts.max_depth_m_ = 1.3;
    InformationPotential<TypeParam> info_pot_far(far_opts);
    for (const InfoMetricType& info_t :
         InformationPotential<TypeParam>::kValidMetric)
    {
      EXPECT_GT(info_pot_near.getMetricThresh(info_t),
                info_pot_far.getMetricThresh(info_t));
      EXPECT_GT(info_pot_near.getMetricThreshApproxVis(info_t),
                info_pot_far.getMetricThreshApproxVis(info_t));
    }
  }

  {
    InfoPotentialOptions narrow_opts;
    narrow_opts.fov_deg_ = 60;
    InformationPotential<TypeParam> info_pot_narrow(narrow_opts);

    InfoPotentialOptions wide_opts;
    wide_opts.fov_deg_ = 120;
    InformationPotential<TypeParam> info_pot_wide(wide_opts);
    for (const InfoMetricType& info_t :
         InformationPotential<TypeParam>::kValidMetric)
    {
      std::cout << kInfoMetricNames.at(info_t) << std::endl;
      std::cout << "- narrorw: " << info_pot_narrow.getMetricThresh(info_t);
      std::cout << "; wide: " << info_pot_wide.getMetricThresh(info_t);
      std::cout << std::endl;
      if (info_t == InfoMetricType::kTrace)
      {
        const double diff = std::fabs(info_pot_wide.getMetricThresh(info_t) -
                                      info_pot_narrow.getMetricThresh(info_t));
        EXPECT_LT(diff / info_pot_wide.getMetricThresh(info_t), 0.1);
      }
      else
      {
        EXPECT_GT(info_pot_wide.getMetricThresh(info_t),
                  info_pot_narrow.getMetricThresh(info_t));
      }
    }
  }
}

TYPED_TEST(InfoPotentialTest, info_potential)
{
  struct PotentialFunc : public aslam::common::NumDiffFunctor<3, 1>
  {
    PotentialFunc(InformationPotential<TypeParam>* info_pot,
                  const InfoMetricType& t)
      : info_pot_(info_pot), info_t_(t)
    {
    }
    bool functional(const typename NumDiffFunctor::InputType& x,
                    typename NumDiffFunctor::ValueType& fvec,
                    typename NumDiffFunctor::JacobianType* Jout) const
    {
      if (Jout)
      {
        double deriv;
        fvec(0, 0) = info_pot_->eval(x(0, 0), info_t_, &deriv);
        (*Jout)(0, 0) = deriv;
        fvec(1, 0) = info_pot_->evalApproxVis(x(0, 0), info_t_, &deriv);
        (*Jout)(1, 0) = deriv;
        fvec(2, 0) = info_pot_->evalApproxVis(x(0, 0), info_t_, &deriv);
        (*Jout)(2, 0) = deriv;
      }
      else
      {
        fvec(0, 0) = info_pot_->eval(x(0, 0), info_t_);
        fvec(1, 0) = info_pot_->evalApproxVis(x(0, 0), info_t_);
        fvec(2, 0) = info_pot_->evalApproxVis(x(0, 0), info_t_);
      }

      return true;
    }
    InformationPotential<TypeParam>* info_pot_;
    InfoMetricType info_t_;
  };

  InfoPotentialOptions options;
  InformationPotential<TypeParam> info_pot(options);
  std::cout << "Information Metric:\n";
  for (const InfoMetricType& info_t :
       InformationPotential<TypeParam>::kValidMetric)
  {
    const double info_val = info_pot.getMetricThresh(info_t);
    EXPECT_GT(info_val, 0.0);
    std::cout << "- " << kInfoMetricNames.at(info_t) << ": " << info_val
              << std::endl;
  }

  constexpr double step_size = 1e-10;
  constexpr double test_tol = 1e-2;
  std::random_device rd;
  std::mt19937 gen(rd());

  for (const InfoMetricType& info_t :
       InformationPotential<TypeParam>::kValidMetric)
  {
    std::cout << "=====> Testing potential for info type "
              << kInfoMetricNames.at(info_t) << std::endl;
    const double thresh = info_pot.getMetricThresh(info_t);
    std::uniform_real_distribution<double> dist(-5 * thresh, 5 * thresh);
    InfoPotentialFunc info_pot_func(thresh, options.val_at_zero_);
    for (int i = 0; i < 50; i++)
    {
      Eigen::Matrix<double, 1, 1> input;
      input(0, 0) = dist(gen);
      double val_potential_func = info_pot_func.eval(input(0, 0));
      double val_info_pot = info_pot.eval(input(0, 0), info_t);
      EXPECT_DOUBLE_EQ(val_potential_func, val_info_pot);
      TEST_JACOBIAN_FINITE_DIFFERENCE(PotentialFunc, input, step_size, test_tol,
                                      &info_pot, info_t);
    }
  }
}

TEST(InfoPotentialFuncTest, info_potential_func)
{
  const double thresh = 10.0;
  const double val_at_zero = 100.0;
  InfoPotentialFunc info_func(thresh, val_at_zero);
  std::random_device rd;
  std::mt19937 gen(rd());

  // zero zone
  {
    std::uniform_real_distribution<double> dist(thresh, 100 * thresh);
    for (int i = 0; i < 100; i++)
    {
      double zero_query = 1.5 * dist(gen);
      double val = info_func.eval(zero_query);
      EXPECT_DOUBLE_EQ(0.0, val);
    }
  }

  // reciprocal
  const double max_quad = info_func.eval(0.0);
  EXPECT_DOUBLE_EQ(max_quad, val_at_zero);
  {
    std::uniform_real_distribution<double> dist(0.0, thresh);
    std::uniform_real_distribution<double> dist_narrorw(0.0, thresh * 0.8);
    for (int i = 0; i < 100; i++)
    {
      double query = dist(gen);
      double val = info_func.eval(query);
      EXPECT_GT(val, 0.0);
      EXPECT_LE(val, max_quad);

      query = dist_narrorw(gen);
      EXPECT_GT(info_func.eval(query), info_func.eval(query * 1.1));
    }
  }

  // linear
  {
    std::uniform_real_distribution<double> dist(-100 * thresh, 0.0);
    for (int i = 0; i < 100; i++)
    {
      double query = dist(gen);
      double val = info_func.eval(query);
      EXPECT_GE(val, max_quad);
      EXPECT_GT(info_func.eval(query - thresh), info_func.eval(query));
    }
  }
}

TEST(InfoPotentialFuncTest, potential_func_grad)
{
  struct PotentialFunc : public aslam::common::NumDiffFunctor<1, 1>
  {
    PotentialFunc(InfoPotentialFunc* info_pot_func)
      : info_pot_func_(info_pot_func)
    {
    }
    bool functional(const typename NumDiffFunctor::InputType& x,
                    typename NumDiffFunctor::ValueType& fvec,
                    typename NumDiffFunctor::JacobianType* Jout) const
    {
      if (Jout)
      {
        double deriv;
        fvec(0, 0) = info_pot_func_->eval(x(0, 0), &deriv);
        (*Jout)(0, 0) = deriv;
      }
      else
      {
        fvec(0, 0) = info_pot_func_->eval(x(0, 0));
      }

      return true;
    }
    InfoPotentialFunc* info_pot_func_;
  };

  constexpr double step_size = 1e-6;
  constexpr double test_tol = 1e-2;

  constexpr double thresh = 10.0;
  constexpr double val_at_zero = 100.0;

  InfoPotentialFunc info_func(thresh, val_at_zero);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> test_dist(-5 * thresh, 5 * thresh);
  for (int i = 0; i < 100; i++)
  {
    Eigen::Matrix<double, 1, 1> input;
    input(0, 0) = test_dist(gen);
    TEST_JACOBIAN_FINITE_DIFFERENCE(PotentialFunc, input, step_size, test_tol,
                                    &info_func);
  }
}

TEST(InfoPotentialFuncTest, save)
{
  constexpr double thresh = 10.0;
  constexpr double val_at_zero = 100.0;

  InfoPotentialFunc info_func(thresh, val_at_zero);
  std::vector<double> samples;
  vi_utils::linspace(-5 * thresh, 5 * thresh, 0.01, &samples);

  Eigen::MatrixXd res;
  res.resize(samples.size(), 2);
  for (size_t i = 0; i < samples.size(); i++)
  {
    res(i, 0) = samples[i];
    res(i, 1) = info_func.eval(samples[i]);
  }

  rpg::save("/tmp/info_pot_res.txt", res);
}

RPG_COMMON_TEST_MAIN
{
}
