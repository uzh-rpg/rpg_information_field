//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "vi_utils/common_utils.h"

#include <glog/logging.h>

namespace vi_utils
{
std::string getBaseName(const std::string& filename)
{
  const std::string separator = "/";
  std::size_t last_separator = filename.find_last_of(separator);
  if (last_separator == std::string::npos)
  {
    return std::string();
  }
  return filename.substr(0, last_separator);
}

void linspace(const double start,
              const double end,
              const size_t N,
              std::vector<double>* samples)
{
  CHECK_GT(end, start);
  CHECK_GT(N, 1u);
  CHECK_NOTNULL(samples);
  samples->clear();
  samples->resize(N);

  (*samples)[0] = start;
  double interval = (end - start) / (N - 1);
  for (size_t i = 1; i < N - 1; i++)
  {
    (*samples)[i] = (*samples)[i - 1] + interval;
  }
  (*samples)[N - 1] = end;

  CHECK_NEAR((*samples)[N - 1], (*samples)[N - 2] + interval, 0.001);
}

void linspace(const double start,
              const double end,
              const double step,
              std::vector<double>* samples)
{
  CHECK_GT(end, start);
  CHECK_GT(step, 0.0);

  double true_end = start;
  size_t N = 0;
  while(1)
  {
    N ++;
    if (true_end > (end-0.0001))
    {
      break;
    }
    true_end += step;
  }

  linspace(start, true_end, N, samples);
}
}
