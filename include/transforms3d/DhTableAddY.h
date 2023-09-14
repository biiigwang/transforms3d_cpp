/**
 * @file     DhTableAddY.h
 * @author   Wang Zhen (wangzhen9623@163.com)
 * @brief    改造DhTable，增加Y轴
 * @version  0.1
 * @date     2023-07-20
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef DHTABLEADDY_H
#define DHTABLEADDY_H
#include <math.h>

#include <iomanip>
#include <ios>
#include <ostream>

#include "IDhTable.h"
#include "transforms3d/transforms3d.h"
namespace transforms3d {
/**
 * @brief    Add Y Denavit-Hartenberg base on Craig version
 *  (alpha_{i-1}, a_{i-1}, theta_i, d_i, l_{i+1})
 *
 * @tparam S type
 */
template <typename S>
class DhRowAddY : public IDhRow<S> {
 public:
  /// @brief alpha_{i-1}: 以 𝑋𝑖−1方向看， 𝑍𝑖−1和 𝑍𝑖间的夹角
  S alpha_i_1;

  /// @brief 沿着 𝑋𝑖−1方向， 𝑍𝑖−1和 𝑍𝑖间的距离(𝑎𝑖>0)
  S a_i_1;

  /// @brief theta_i: 以 𝑍𝑖方向看， 𝑋𝑖−1和 𝑋𝑖间的夹角
  S theta_i;

  /// @brief 沿着 𝑍𝑖方向， 𝑋𝑖−1和 𝑋𝑖间的距离
  S d_i;

  /// @brief 沿着 Y𝑖+1方向， Z𝑖+1和 Z𝑖间的距离
  S l_i1;

 private:
  /// @brief alpha_{i-1}的转换矩阵
  Eigen::Matrix4<S> m_T_alpha;

  /// @brief a_{i-1}的转换矩阵
  Eigen::Matrix4<S> m_T_a;

  /// @brief theta_i的转换矩阵
  Eigen::Matrix4<S> m_T_theta;

  /// @brief d_i的转换矩阵
  Eigen::Matrix4<S> m_T_d;

  /// @brief l_{i+1}的转换矩阵
  Eigen::Matrix4<S> m_T_l;

  /// @brief 转换矩阵
  Eigen::Matrix4<S> m_T;

 public:
  /// @brief 构造函数
  DhRowAddY() = default;
  DhRowAddY(S alpha_i_1, S a_i_1, S theta_i, S d_i, S l_i1)
      : alpha_i_1(alpha_i_1),
        a_i_1(a_i_1),
        theta_i(theta_i),
        d_i(d_i),
        l_i1(l_i1) {}

  DhTableType getType() override { return DhTableType::ADDY; }

  /// @brief 计算alpha_{i-1}的转换矩阵
  void computeT_alpha() {
    // clang-format off
    S c_a = cos(TransForms<S>::Degrees(alpha_i_1));
    S s_a = sin(TransForms<S>::Degrees(alpha_i_1));
    m_T_alpha << 1, 0, 0, 0,
                 0, c_a, -s_a, 0,
                 0, s_a,  c_a, 0,
                 0, 0, 0, 1;
    // clang-format on
  };

  /// @brief 计算a_{i-1}的转换矩阵
  void computeT_a() {
    // clang-format off
    m_T_a << 1, 0, 0, a_i_1,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    // clang-format on
  };

  /// @brief 计算theta_i的转换矩阵
  void computeT_theta() {
    // clang-format off
    S c_t = cos(TransForms<S>::Degrees(theta_i));
    S s_t = sin(TransForms<S>::Degrees(theta_i));
    m_T_theta << c_t, -s_t, 0, 0,
                 s_t,  c_t, 0, 0,
                   0,    0, 1, 0,
                   0,    0, 0, 1;
    // clang-format on
  };

  /// @brief 计算d_i的转换矩阵
  void computeT_d() {
    // clang-format off
    m_T_d << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, d_i,
             0, 0, 0, 1;
    // clang-format on
  };

  /// @brief 计算l_{i+1}的转换矩阵
  void computeT_l() {
    // clang-format off
    m_T_l << 1, 0, 0, 0,
             0, 1, 0, l_i1,
             0, 0, 1, 0,
             0, 0, 0, 1;
    // clang-format on
  };

  /// @brief 计算转换矩阵
  void computeTransform() {
    // clang-format off
    // TODO：此处待优化
    m_T = m_T_alpha * m_T_a * m_T_theta * m_T_d * m_T_l; 
    // S c_t = cos(theta_i);
    // S s_t = sin(theta_i);
    // S c_a = cos(alpha_i_1);
    // S s_a = sin(alpha_i_1);

    // m_T << c_t, -s_t, 0, a_i_1,
    //        s_t * c_a, c_t * c_a, -s_a, -s_a * d_i,
    //        s_t * s_a, c_t * s_a, c_a, c_a * d_i,
    //        0, 0, 0, 1;
    // clang-format on    
  };

  /// @brief 获取a_{i-1}的转换矩阵
  Eigen::Matrix4<S> T_alpha() const { return m_T_alpha; }

  /// @brief 获取a_{i-1}的转换矩阵
  Eigen::Matrix4<S> T_a() const { return m_T_a; }

  /// @brief 获取theta_i的转换矩阵
  Eigen::Matrix4<S> T_theta() const { return m_T_theta; }

  /// @brief 获取d_i的转换矩阵
  Eigen::Matrix4<S> T_d() const { return m_T_d; }

  /// @brief 获取l_{i+1}的转换矩阵
  Eigen::Matrix4<S> T_l() const { return m_T_l; }

  Eigen::Matrix4<S> T() const { return m_T; }
  void set_T(const Eigen::Matrix4<S> &T) { m_T = T; }

  /// @brief 获取转换矩阵
  Eigen::Matrix4<S> getTransform() const override  { return m_T; }

  /// @brief 初始化
  void build() override {
    computeT_alpha();
    computeT_a();
    computeT_theta();
    computeT_d();
    computeT_l();
    computeTransform();
  }

  /// @brief toString
  std::string toString() const override {
    std::stringstream ss;
    ss << std::right << std::fixed
     << "alpha:"<< std::setprecision(std::abs(alpha_i_1)<1e-6?0:2) << std::setw(7) <<std::setfill(' ') << alpha_i_1 << ","
     << "a:"    << std::setprecision(std::abs(a_i_1)<1e-6?0:2) << std::setw(7) <<std::setfill(' ') << a_i_1 << ","
     << "theta:"<< std::setprecision(std::abs(theta_i)<1e-6?0:2) << std::setw(7) <<std::setfill(' ') << theta_i << ","
     << "d:"    << std::setprecision(std::abs(d_i)<1e-6?0:2) << std::setw(7) <<std::setfill(' ') << d_i << ","
     << "l:"    << std::setprecision(std::abs(l_i1)<1e-6?0:2) << std::setw(7) <<std::setfill(' ') << l_i1;
    return ss.str();
  }

  bool operator==(const DhRowAddY<S> &other) const {
    return alpha_i_1 == other.alpha_i_1
        && a_i_1 == other.a_i_1
        && theta_i == other.theta_i
        && d_i == other.d_i
        && m_T_alpha == other.m_T_alpha
        && m_T_a == other.m_T_a
        && m_T_theta == other.m_T_theta
        && m_T_d == other.m_T_d
        && m_T_l == other.m_T_l
        && m_T == other.m_T;
  }
  bool operator!=(const DhRowAddY<S> &other) const { return !(*this == other); }


};

using DhRowAddYf = DhRowAddY<float>;
using DhRowAddYd = DhRowAddY<double>;  
}

#endif // DHTABLEADDY_H
