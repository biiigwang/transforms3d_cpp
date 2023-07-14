/**
 * @file     DhTableCraig.cpp
 * @author   Wang Zhen (wangzhen9623@163.com)
 * @brief    
 * @version  0.1
 * @date     2023-07-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <transforms3d/DhTableCraig.h>

namespace transforms3d {

  template 
  class DhRowCraig<double>;

  template 
  class DhRowCraig<float>;
}