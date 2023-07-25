/*
 * @Descripttion:
 * @Author: sangxin
 * @Date: 2021-05-01 21:04:19
 * @LastEditTime: 2021-05-02 23:39:20
 */
#ifndef TRANS_FORMS_GROUP_INL_H
#define TRANS_FORMS_GROUP_INL_H

#include <transforms3d/transforms3d.h>

#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>

namespace transforms3d {

extern template class TransFormsGroup<double>;

extern template class TransFormsGroup<float>;

template <typename S>
Matrix4<S> TransFormsGroup<S>::getTransForm(const std::string &start,
                                            const std::string &end) {
  Matrix4<S> matrix;
  matrix.setIdentity();  //单位矩阵
  std::vector<std::string> paths = findPath(start, end);
  if (paths.size() <= 1) {
    return matrix;
  }
  for (int i = 0; i < paths.size() - 1; i++) {
    for (Child c : tfg[paths[i]]) {
      if (c.name == paths[i + 1]) {
        matrix *= c.t;
      }
    }
  }
  return matrix;
}

template <typename S>
Matrix4<S> TransFormsGroup<S>::pushTransForm(const std::string &parent,
                                             const std::string &child, S &x,
                                             S &y, S &z, S &rx, S &ry, S &rz) {
  return pushTransForm(parent, child,
                       TransForms<S>::ComposeEuler(x, y, z, rx, ry, rz));
}

template <typename S>
Matrix4<S> TransFormsGroup<S>::pushTransForm(const std::string &parent,
                                             const std::string &child, S &x,
                                             S &y, S &z, S &rx, S &ry, S &rz,
                                             S &rw) {
  Vector3<S> positon(x, y, z);
  Quaternion<S> quat(rx, ry, rz, rw);
  return pushTransForm(parent, child, TransForms<S>::Compose(positon, quat));
}

template <typename S>
Matrix4<S> TransFormsGroup<S>::pushTransForm(const std::string &parent,
                                             const std::string &child,
                                             Matrix4<S> matrix) {
  bool has_parent_node = true, has_child_node = true;
  if (tfg.find(parent) == tfg.end()) has_parent_node = false;
  if (tfg.find(child) == tfg.end()) has_child_node = false;

  // 分情况判断
  if (has_parent_node) {
    std::vector<Child> childs = tfg[parent];
    bool has_child = false;
    for (int i = 0; i < childs.size(); i++) {
      if (childs[i].name == child) {
        has_child = true;
        // 更新已有的矩阵
        tfg[parent][i].t = matrix;
      }
    }
    if (!has_child) {
      // 没有这个孩子，添加孩子
      Child new_child;
      new_child.name = child;
      new_child.t = matrix;
      tfg[parent].push_back(new_child);
    }
  } else {
    // 添加一个节点
    Child new_child;
    new_child.name = child;
    new_child.t = matrix;
    std::vector<Child> new_childs;
    new_childs.push_back(new_child);
    //添加一个父节点
    tfg[parent] = new_childs;
  }

  // 分情况判断
  if (has_child_node) {
    std::vector<Child> childs = tfg[child];
    bool has_child = false;
    for (int i = 0; i < childs.size(); i++) {
      if (childs[i].name == parent) {
        has_child = true;
        // 更新已有的矩阵
        tfg[child][i].t = matrix.inverse();
      }
    }
    if (!has_child) {
      // 没有这个孩子，添加孩子
      Child new_child;
      new_child.name = parent;
      new_child.t = matrix.inverse();
      tfg[child].push_back(new_child);
    }
  } else {
    // 添加一个节点
    Child new_child;
    new_child.name = parent;
    new_child.t = matrix.inverse();
    std::vector<Child> new_childs;
    new_childs.push_back(new_child);
    tfg[child] = new_childs;
  }
  return matrix;
}

template <typename S>
std::string TransFormsGroup<S>::toString() {
  std::string str;
  typename std::map<std::string, std::vector<Child>>::iterator iter;
  for (iter = tfg.begin(); iter != tfg.end(); iter++) {
    // cout << "node:" << iter->first << endl;
    str += "node:" + iter->first + "\n";
    for (int i = 0; i < iter->second.size(); i++) {
      // cout <<"\tchild" << i << " : " << iter->second[i].name << endl;
      // cout <<"\t\tmatrix : ";
      VectorX<S> vxd = TransForms<S>::H2EulerAngle(iter->second[i].t);
      // cout<<fixed<<setprecision(5)<<left << setw(14)<<vxd[0]<<
      // setw(14)<<vxd[1]<< setw(14)<<vxd[2]<< setw(14)<<vxd[3]<<
      // setw(14)<<vxd[4]<< setw(14)<<vxd[5]<<endl;
      str += "\tchild: " + iter->second[i].name +
             "\n\t\tmatrix :  " + std::to_string(vxd[0]) + "," +
             std::to_string(vxd[1]) + "," + std::to_string(vxd[2]) + "," +
             std::to_string(vxd[5]) + "," + std::to_string(vxd[4]) + "," +
             std::to_string(vxd[3]) + "\n";
    }
    std::cout << std::endl;
    str += "\n";
  }
  return str;
}

template <typename S>
std::vector<std::string> TransFormsGroup<S>::findPath(const std::string &start,
                                                      const std::string &end) {
  std::deque<TransFormsGroup::Path> que;
  std::vector<std::string> researched = {start};
  if (start == end) return researched;
  for (Child c : tfg[start]) {
    Path path = {c.name, {start, c.name}};
    que.push_back(path);
  }
  while (!que.empty()) {
    Path path = que.front();
    // cout<<"-----------------------------"<<endl;;
    // cout<<path.name<<":";
    // for(std::string p:path.path){
    //     cout<<"   "<<p;
    // }
    // cout<<"researched"<<":";
    // for(std::string r:researched){
    //     cout<<"   "<<r;
    // }
    // cout<<endl;
    // cout<<"before quesize:"<<que.size()<<endl;

    que.pop_front();

    if (std::find(researched.begin(), researched.end(), path.name) ==
        researched.end()) {
      // cout<<"not find path.name"<<endl;
      if (path.name == end) {
        // cout<<"fing node"<<endl;
        return path.path;
      } else {
        for (Child c : tfg[path.name]) {
          // cout<<"prepare add "<<c.name<<endl;
          if (std::find(researched.begin(), researched.end(), c.name) ==
              researched.end()) {
            std::vector<std::string> paths = {c.name};
            paths.insert(paths.begin(), path.path.begin(), path.path.end());
            Path temPath = {c.name, paths};
            que.push_back(temPath);
            // cout<<"added "<<c.name<<endl;
          }
        }
      }
    }
    // cout<<"quesize:"<<que.size()<<endl;
  }

  return researched;
}

/**
 * @brief 将点云从一个坐标系转换到另外一个坐标系
 *
 * @param base
 * @param point
 * @param target
 * @return std::vector<Vector3<S>>
 */

template <typename S>
std::vector<Vector3<S>> TransFormsGroup<S>::getTransWithPointCloud(
    const std::string &base, std::vector<Vector3<S>> &points,
    const std::string &target) {
  Matrix4<S> base2target = getTransForm(target, base);
  Matrix3<S> rotate;
  Vector3<S> transpose;
  TransForms<S>::HDecompose(base2target, rotate, transpose);
  for (uint64_t i = 0; i < points.size(); i++) {
    Vector3<S> temp_v3d = rotate * points[i];
    points[i] = temp_v3d + transpose;
  }
  return points;
}
}  // namespace transforms3d

#endif  // TRANS_FORMS_GROUP_INL_H
