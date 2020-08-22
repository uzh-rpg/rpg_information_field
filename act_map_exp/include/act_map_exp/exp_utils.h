#pragma once

#include <glog/logging.h>

#include <rpg_common/pose.h>
#include <rpg_common/eigen_type.h>
#include <rpg_common/save.h>

#include <unrealcv_bridge/ue_utils.hpp>

namespace act_map_exp
{
struct TimeValues
{
  std::vector<double> times;
  std::vector<double> values;
  inline void resize(const size_t N)
  {
    times.resize(N, 0.0);
    values.resize(N, 0.0);
  }
  inline void clear()
  {
    times.clear();
    values.clear();
  }
  TimeValues()
  {
  }
  inline void check()
  {
    CHECK_EQ(times.size(), values.size());
  }

  void save(const std::string& fn) const;
};

inline void skew(const Eigen::Vector3d& v, Eigen::Matrix3d* skew)
{
  skew->setZero();

  (*skew)(0, 1) = -v(2);
  (*skew)(0, 2) = v(1);

  (*skew)(1, 0) = v(2);
  (*skew)(1, 2) = -v(0);

  (*skew)(2, 0) = -v(1);
  (*skew)(2, 1) = v(0);
}

inline std::string asStr(const bool val)
{
  return val ? std::string("TRUE") : std::string("FALSE");
}

inline std::string asStr(const std::string& str)
{
  return str.empty() ? std::string("EMPTY STRING") : str;
}

inline void sampleIndices(const size_t sample_N, const size_t total,
                          std::vector<size_t>* sampled_indices)
{
  const size_t step =
      static_cast<size_t>(std::floor((total - 1) * 1.0 / (sample_N - 1)));
  for (size_t i = 0; i < total; i += step)
  {
    sampled_indices->emplace_back(i);
  }
  if (sampled_indices->back() != (total - 1))
  {
    sampled_indices->back() = total - 1;
  }
}

template <int DimV>
inline void jacVecCrossProd(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                            const Eigen::Matrix<double, 3, DimV>& da_dv,
                            const Eigen::Matrix<double, 3, DimV>& db_dv,
                            Eigen::Matrix<double, 3, DimV>* dout_dv)
{
  // y(v) = a(v).cross(b(v))
  // --> dy/dv
  Eigen::Matrix3d skew_a, skew_b;
  skew(a, &skew_a);
  skew(b, &skew_b);

  (*dout_dv) = skew_a * db_dv - skew_b * da_dv;
}

void jacVector3Normalization(const Eigen::Vector3d& v, Eigen::Matrix3d* jac);

// recovering rotation from acceleration and yaw:
// http://www-personal.acfr.usyd.edu.au/spns/cdm/papers/Mellinger.pdf
void quadAccYawToRwb(const Eigen::Vector3d& acc_w, const double yaw_rad,
                     rpg::Rotation::RotationMatrix* rot_mat,
                     rpg::Matrix93* drotmat_dacc, rpg::Matrix91* drotmat_dyaw);

inline void dRotGdR(const rpg::Rotation::RotationMatrix& rotmat,
                    rpg::Matrix39* drotg_dR)
{
  const Eigen::Ref<const Eigen::RowVectorXd> da2_ddr1 = rotmat.row(0);
  const Eigen::Ref<const Eigen::RowVectorXd> da0_ddr2 = rotmat.row(1);
  const Eigen::Ref<const Eigen::RowVectorXd> da1_ddr0 = rotmat.row(2);

  drotg_dR->setZero();
  drotg_dR->block<1, 3>(0, 6) = da0_ddr2;
  drotg_dR->block<1, 3>(1, 0) = da1_ddr0;
  drotg_dR->block<1, 3>(2, 3) = da2_ddr1;
}

template <typename T, int N>
inline void squareMatMulJac(const Eigen::Matrix<T, N, N>& B,
                            Eigen::Matrix<T, N * N, N * N>* dAB_dA)
{
  dAB_dA->setZero();
  int cnt = 0;
  for (int ri = 0; ri < N; ri++)
  {
    for (int ci = 0; ci < N; ci++)
    {
      dAB_dA->template block<1, N>(cnt, ri * N) = B.col(ci).transpose();
      cnt++;
    }
  }
}

// b = Ax
template <typename T, int N>
inline void matVecMulJac(const Eigen::Matrix<T, N, 1>& x,
                         Eigen::Matrix<T, N, N * N>* db_dA)
{
  db_dA->setZero();
  for (int i = 0; i < x.rows(); i++)
  {
    db_dA->template block<1, N>(i, i * N) = x.transpose();
  }
}

// pack and unpack vectors for the interface
inline void packVecVecXd(const std::vector<Eigen::VectorXd>& vec,
                         std::vector<double>* flat)
{
  CHECK_NOTNULL(flat);

  flat->reserve(flat->size() + vec.size() * vec.front().size());
  for (size_t vec_i = 0; vec_i < vec.size(); vec_i++)
  {
    const Eigen::VectorXd& ele = vec[vec_i];
    for (int ele_i = 0; ele_i < ele.size(); ele_i++)
    {
      flat->push_back(ele[ele_i]);
    }
  }
}

inline void packVecVecXd(const std::vector<Eigen::VectorXd>& vec,
                         const size_t offset, double* flat)
{
  CHECK_NOTNULL(flat);

  size_t cnt = 0;
  for (size_t vec_i = 0; vec_i < vec.size(); vec_i++)
  {
    const Eigen::VectorXd& ele = vec[vec_i];
    for (int ele_i = 0; ele_i < ele.size(); ele_i++)
    {
      *(flat + offset + cnt) = ele[ele_i];
      cnt++;
    }
  }
}

inline void unpackVecToVecVecXd(const double* flat, const size_t flat_start,
                                std::vector<Eigen::VectorXd>* vec)
{
  CHECK_NOTNULL(vec);
  CHECK(!vec->empty());
  size_t flat_i = 0;
  for (size_t vec_i = 0; vec_i < vec->size(); vec_i++)
  {
    Eigen::VectorXd& ele = (*vec)[vec_i];
    for (int ele_i = 0; ele_i < ele.size(); ele_i++)
    {
      ele[ele_i] = *(flat + flat_start + flat_i);
      flat_i++;
    }
  }
}

inline void accumulate(const std::vector<Eigen::VectorXd>& to_add,
                       const double multiplier,
                       std::vector<Eigen::VectorXd>* sum)
{
  CHECK_EQ(to_add.size(), sum->size());
  for (size_t i = 0; i < to_add.size(); i++)
  {
    (*sum)[i] += (to_add[i] * multiplier);
  }
}

inline void setZero(std::vector<Eigen::VectorXd>* p)
{
  for (auto& v : *p)
  {
    v.setZero();
  }
}

inline void setRandom(std::vector<Eigen::VectorXd>* p)
{
  for (auto& v : *p)
  {
    v.setRandom();
  }
}

template <int Row, int Col>
void saveStampedEigenMatrices(
    const std::vector<double>& times,
    const rpg::Aligned<std::vector, Eigen::Matrix<double, Row, Col>>& matrices,
    const std::string& abs_fn, const std::string& header_pref = std::string(""))
{
  size_t N = times.size();
  CHECK_EQ(N, matrices.size());
  const int flat_size = matrices.front().size();
  const int nrow = matrices.front().rows();
  const int ncol = matrices.front().cols();

  Eigen::MatrixXd data;
  data.resize(N, flat_size + 1);
  for (int i = 0; i < static_cast<int>(N); i++)
  {
    data(i, 0) = times[static_cast<size_t>(i)];
    const Eigen::Matrix<double, Row, Col>& mat_i =
        matrices[static_cast<size_t>(i)];
    CHECK_EQ(flat_size, mat_i.size());
    CHECK_EQ(nrow, mat_i.rows());
    CHECK_EQ(ncol, mat_i.cols());
    int ele_cnt = 0;
    for (int ri = 0; ri < mat_i.rows(); ri++)
    {
      for (int ci = 0; ci < mat_i.cols(); ci++)
      {
        data(i, 1 + ele_cnt++) = mat_i(ri, ci);
      }
    }
  }
  const std::string header = header_pref + std::to_string(N) +
                             " entries: " + "time; mat of size: " +
                             std::to_string(matrices.front().rows()) + " x " +
                             std::to_string(matrices.front().cols());
  rpg::save(abs_fn, data, rpg::EigenSpaceSeparatedFmt, header);
}

template <typename T>
void saveStampedScalars(const std::vector<double>& times,
                        const std::vector<T>& values, const std::string& abs_fn)
{
  CHECK_EQ(values.size(), times.size());
  const int N = static_cast<int>(times.size());
  Eigen::Matrix<double, Eigen::Dynamic, 2> data;
  data.resize(N, Eigen::NoChange);
  for (int i = 0; i < N; i++)
  {
    data(i, 0) = times[static_cast<size_t>(i)];
    data(i, 1) = values[static_cast<size_t>(i)];
  }
  rpg::save(abs_fn, data, rpg::EigenSpaceSeparatedFmt);
}

void saveStampedPoses(const std::vector<double>& times,
                      const rpg::PoseVec& poses, const std::string& abs_fn);

void saveStampedPoses(const std::vector<double>& times,
                      const unrealcv_bridge::UEPoseVec& poses,
                      const std::string& abs_fn);

double getPathLength(const rpg::PoseVec& poses);

}  // namespace act_map_exp
