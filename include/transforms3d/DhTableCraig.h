/**
 * @file     DHTable.h
 * @author   Wang Zhen (wangzhen9623@163.com)
 * @brief    DH参数表相关
 * @version  0.1
 * @date     2023-07-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef DHTABLE_H
#define DHTABLE_H
#include <math.h>

#include <iomanip>
#include <ios>
#include <ostream>

#include "IDhTable.h"
#include "transforms3d/transforms3d.h"

namespace transforms3d {

/**
 * @brief    Craig version Denavit-Hartenberg
 *  (alpha_{i-1}, a_{i-1}, theta_i, d_i)
 *
 * @tparam S type
 */
template <typename S>
class DhRowCraig : public IDhRow<S> {
 public:
  /// @brief alpha_{i-1}: 以 𝑋𝑖−1方向看， 𝑍𝑖−1和 𝑍𝑖间的夹角
  S alpha_i_1;

  /// @brief 沿着 𝑋𝑖−1方向， 𝑍𝑖−1和 𝑍𝑖间的距离(𝑎𝑖>0)
  S a_i_1;

  /// @brief theta_i: 以 𝑍𝑖方向看， 𝑋𝑖−1和 𝑋𝑖间的夹角
  S theta_i;

  /// @brief 沿着 𝑍𝑖方向， 𝑋𝑖−1和 𝑋𝑖间的距离
  S d_i;

 private:
  /// @brief alpha_{i-1}的转换矩阵
  Eigen::Matrix4<S> m_T_alpha;

  /// @brief a_{i-1}的转换矩阵
  Eigen::Matrix4<S> m_T_a;

  /// @brief theta_i的转换矩阵
  Eigen::Matrix4<S> m_T_theta;

  /// @brief d_i的转换矩阵
  Eigen::Matrix4<S> m_T_d;

  /// @brief 转换矩阵
  Eigen::Matrix4<S> m_T;

 public:
  /// @brief 构造函数
  DhRowCraig() = default;
  DhRowCraig(S alpha_i_1, S a_i_1, S theta_i, S d_i)
      : alpha_i_1(alpha_i_1), a_i_1(a_i_1), theta_i(theta_i), d_i(d_i) {}

  DhTableType getType() override { return DhTableType::CRAIG; }

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

  /// @brief 计算转换矩阵
  void computeTransform() {
    // clang-format off
    m_T = m_T_alpha * m_T_a * m_T_theta * m_T_d; 
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
    computeTransform();
  }

  /// @brief toString
  std::string toString() const override {
    std::stringstream ss;
    ss << std::right
     << "alpha:" << std::setw(4)<< alpha_i_1 << ","
     << "a:" << std::setw(4)<< a_i_1 << ","
     << "theta:" << std::setw(4)<< theta_i << ","
     << "d:" << std::setw(4)<< d_i << ";";
    return ss.str();
  }


};

using DhRowCraigf = DhRowCraig<float>;
using DhRowCraigd = DhRowCraig<double>;
}  // namespace transforms3d

#endif  // DHTABLE_H
