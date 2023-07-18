/**
 * @file     IDhTable.h
 * @author   Wang Zhen (wangzhen9623@163.com)
 * @brief    D-H table interface
 * @version  0.1
 * @date     2023-07-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef IDHTABLE_H
#define IDHTABLE_H

#include <Eigen/Core>
#include <map>
#include <memory>
#include <string>

#include "transforms3d/transforms3d.h"

namespace transforms3d {

enum class DhTableType { CRAIG, STD };

template <typename S>
class IDhRow {
 public:
  virtual ~IDhRow(){};

  /// @brief 获取D-H表的一行 的类型
  virtual DhTableType getType() = 0;

  /// @brief 获取D-H表的一行 的转换矩阵
  virtual void build() = 0;

  /// @brief 获取相对于父节点的坐标变换
  virtual Eigen::Matrix4<S> getTransform() const = 0;

  /// @brief toString
  virtual std::string toString() const = 0;
};

}  // namespace transforms3d

#endif  // IDHTABLE_H
