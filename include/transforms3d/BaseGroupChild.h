/**
 * @file     BaseGroupChild.h
 * @author   Wang Zhen (wangzhen9623@163.com)
 * @brief
 * @version  0.1
 * @date     2023-07-13
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef BASEGROUPCHILD_H
#define BASEGROUPCHILD_H

#include <memory>
#include <string>

namespace transforms3d {

class BaseGroupChild {
 public:
  std::string name;

 public:
  explicit BaseGroupChild(const std::string& name) : name(name){};
  virtual ~BaseGroupChild(){};
};

class IGroupChildFactory {
 public:
  virtual ~IGroupChildFactory(){};
  virtual std::unique_ptr<BaseGroupChild> create(const std::string& m_name) = 0;
};
}  // namespace CDC

#endif  // BASEGROUPCHILD_H
