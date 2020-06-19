//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map/sampler.h"

#include <rpg_common/test_main.h>

using namespace act_map;

TEST(SamplerTest, testUniform)
{
  const double xrange = 4;
  const double yrange = 4;
  const double zrange = 2;
  const double step = 0.2;

  std::vector<double> xvalues, yvalues, zvalues;

  utils::generateUniformPointsWithin(step,
                                     -xrange / 2,
                                     xrange / 2,
                                     -yrange / 2,
                                     yrange / 2,
                                     -zrange / 2,
                                     zrange / 2,
                                     &xvalues,
                                     &yvalues,
                                     &zvalues);
  EXPECT_GT(xvalues.size(), 0u);
  EXPECT_GT(yvalues.size(), 0u);
  EXPECT_GT(zvalues.size(), 0u);

  EXPECT_DOUBLE_EQ(-xrange / 2, xvalues[0]);
  for (size_t i = 0; i < xvalues.size() - 1u; i++)
  {
    EXPECT_NEAR(step, xvalues[i + 1] - xvalues[i], 1e-5);
  }

  EXPECT_DOUBLE_EQ(-yrange / 2, yvalues[0]);
  for (size_t i = 0; i < yvalues.size() - 1u; i++)
  {
    EXPECT_NEAR(step, yvalues[i + 1] - yvalues[i], 1e-5);
  }

  EXPECT_DOUBLE_EQ(-zrange / 2, zvalues[0]);
  for (size_t i = 0; i < zvalues.size() - 1u; i++)
  {
    EXPECT_NEAR(step, zvalues[i + 1] - zvalues[i], 1e-5);
  }

  rpg::PositionVec points;
  utils::generateUniformPointsWithin(step,
                                     -xrange / 2,
                                     xrange / 2,
                                     -yrange / 2,
                                     yrange / 2,
                                     -zrange / 2,
                                     zrange / 2,
                                     &points);
  EXPECT_EQ(points.size(), xvalues.size() * yvalues.size() * zvalues.size());

  rpg::PositionVec points2;
  utils::generateUniformPointsWithin(step, xrange, yrange, zrange, &points);
  for (size_t i = 0; i < points2.size(); i++)
  {
    EXPECT_TRUE(points[i].isApprox(points2[i]));
  }
}

RPG_COMMON_TEST_MAIN
{
}
