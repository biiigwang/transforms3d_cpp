/*
 * @Descripttion: TransForms Header
 * @Author: sangxin
 * @Date: 2021-05-01 21:04:19
 * @LastEditTime: 2021-05-31 14:04:19
 */
#ifndef TRANSFIRMS3D_TRANSFORMS_H_
#define TRANSFIRMS3D_TRANSFORMS_H_

#include <math.h>

#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <stack>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/Geometry"
using namespace Eigen;

namespace transforms3d {

template <typename S>
class TransForms {
 private:
 public:
  /*---------------------------------------角度弧度转换----------------------------------------*/
  /**
   * @brief: 角度转为弧度
   * @param {S} angle 角度值
   * @return 返回对应弧度值,一般在-3.14~3.14之间
   */
  static S Degrees(S angle);
  /**
   * @brief: 弧度转为角度
   * @param {S} degrees 弧度值
   * @return 返回对应的角度值，一般在-180~180之间
   */
  static S Angle(S degrees);

  /*---------------------------------------欧拉角部分---------------------------*/
  /**
   * @brief: 角度制欧拉角转旋转矩阵,此函数默认的旋转顺序是x-y-z.
   * @param {S} rx 绕x轴的旋转.
   * @param {S} ry 绕y轴的旋转.
   * @param {S} rz 绕z轴的旋转.
   * @return {Matrix3<S>}  返回3✖3的旋转矩阵.
   */
  static Matrix3<S> EulerAngle2Mat(S rx, S ry, S rz);

  /**
   * @brief: 欧拉角转旋转矩阵
   * @param {Vector3<S>} eular 欧拉角rx,ry,rz
   * @return {Matrix3<S>} 返回3✖3的旋转矩阵.
   */
  static Matrix3<S> EulerAngle2Mat(Vector3<S> eular);

  /**
   * @brief:欧拉角转四元数
   * @param {S} rx 绕x轴的旋转
   * @param {S} ry 绕y轴的旋转
   * @param {S} rz 绕z轴的旋转
   * @return {Quaternion<S>} 返回对应的四元数
   */
  static Quaternion<S> Euler2Quat(S rx, S ry, S rz);

  /**
   * @brief: 角度制欧拉角转四元数
   * @param {S} rx 绕x轴的旋转
   * @param {S} ry 绕y轴的旋转
   * @param {S} rz 绕z轴的旋转
   * @return {Quaternion<S>} 返回对应的四元数
   */
  static Quaternion<S> EulerAngle2Quat(S rx, S ry, S rz);

  /**
   * @brief: 旋转矩阵转欧拉角(弧度制)
   * @param {Matrix3<S>} 3✖3的旋转矩阵
   * @return {Vector3<S>} 欧拉角
   */
  static Vector3<S> Mat2Euler(Matrix3<S> mat);

  /**
   * @brief: 欧拉角转旋转矩阵
   * @param {S} rx 绕x轴的旋转
   * @param {S} ry 绕y轴的旋转
   * @param {S} rz 绕z轴的旋转
   * @return {*}
   */
  static Matrix3<S> Euler2Mat(S rx, S ry, S rz);

  /**
   * @brief: 旋转矩阵转角度制欧拉角(角度制)
   * @param {Matrix3<S>} 3✖3的旋转矩阵
   * @return {Vector3<S>}  欧拉角
   */
  static Vector3<S> Mat2EulerAngle(Matrix3<S> mat);

  /*---------------------------------------四元数部分----------------------------------------*/
  /**
   * @brief: 四元数转旋转矩阵
   * @param {Quaternion<S>} 四元数
   * @return {Matrix3<S>} 对应的旋转矩阵
   */
  static Matrix3<S> Quat2Mat(Quaternion<S> quat);

  /**
   * @brief: 四元数转欧拉角
   * @param {Quaternion<S>} 四元数
   * @return {Vector3<S>} 对应的欧拉角
   */
  static Vector3<S> Quat2Eular(Quaternion<S> quat);

  /**
   * @brief: 四元数转弧度制欧拉角(角度制)
   * @param {Quaternion<S>} 四元数
   * @return {Vector3<S>} 对应的欧拉角
   */
  static Vector3<S> Quat2EularAngle(Quaternion<S> quat);

  /**
   * @brief: 旋转矩阵转四元数
   * @param {Matrix3<S>} 3✖3的旋转矩阵
   * @return {Quaternion<S>} 对应的四元数
   */
  static Quaternion<S> Mat2Quat(Matrix3<S> mat);

  /*---------------------------------------齐次矩阵部分----------------------------------------*/
  /**
   * @brief: 通过位置和欧拉角合成一个齐次矩阵
   * @param {Vector3<S>} positon 平移位置
   * @param {Vector3<S>} rotEular  旋转变换(欧拉角形式)
   * @return {*}
   */
  static Matrix4<S> Compose(Vector3<S> positon, Vector3<S> rotEular);

  /**
   * @brief: 通过位置和四元数合成一个齐次矩阵
   * @param {Vector3<S>} positon 平移位置
   * @param {Quaternion<S>} quat 四元数
   * @return {Matrix4<S>} 齐次矩阵
   */
  static Matrix4<S> Compose(Vector3<S> positon, Quaternion<S> quat);

  /**
   * @brief: 通过三个位置和三个欧拉角合成一个齐次矩阵
   * @param {S} x 沿x轴的平移
   * @param {S} y 沿y轴的平移
   * @param {S} z 沿z轴的平移
   * @param {S} rx 绕x轴的旋转
   * @param {S} ry 绕y轴的旋转
   * @param {S} rz 绕z轴的旋转
   * @return {Matrix4<S>} 返回4✖4的齐次变换矩阵
   */
  static Matrix4<S> ComposeEuler(S x, S y, S z, S rx, S ry, S rz);

  /**
   * @brief:  将齐次矩阵转换成平移和欧拉角形式，方便理解
   * @param {Matrix4<S>} 4✖4的齐次变换矩阵
   * @return {VectorX<S>} x,y,z,rx,ry,rz
   */
  static VectorX<S> H2EulerAngle(Matrix4<S> t);

  /**
   * @brief 将齐次矩阵分解为位置和旋转矩阵
   *
   * @param t 矩阵
   * @param rotate 旋转矩阵
   * @param position 位置
   * @return Matrix3<S>
   */
  static Matrix3<S> HDecompose(Matrix4<S> t, Matrix3<S> &rotate,
                               Vector3<S> &position);

  TransForms(/* args */) = default;
  ~TransForms() = default;
};

// typedef TransForms<float> TransFormsf;
// typedef TransForms<double> TransFormsd;

using TransFormsf = TransForms<float>;
using TransFormsd = TransForms<double>;

template <typename S>
class TransFormsGroup {
 private:
  struct Child {
    std::string name;
    Matrix4<S> t;
  };
  struct Path {
    std::string name;
    std::vector<std::string> path;
  };
  std::map<std::string, std::vector<Child>> tfg;

 public:
  /**
   * @brief: 将存储的节点数据按照字符串形式输出
   * @param {*} 无
   * @return {std::string}  字符串
   */
  std::string toString();

  /**
   * @brief: 查找两个节点之间的路径
   * @param {const std::string&} start  开始节点名字
   * @param {const std::string&} end  结束节点名字
   * @return {std::vector<std::string> }  路径数组
   */
  std::vector<std::string> findPath(const std::string &start,
                                    const std::string &end);
  /**
   * @brief 将点云从一个坐标系转换到另外一个坐标系
   *
   * @param base
   * @param point
   * @param target
   * @return std::vector<Vector3<S>>
   */
  std::vector<Vector3<S>> getTransWithPointCloud(
      const std::string &base, std::vector<Vector3<S>> &points,
      const std::string &target);
  /**
   * @brief:  获取两个相关节点的变换关系
   * @param {const std::string&} start  开始节点名字
   * @param {const std::string&} end  结束节点名字
   * @return {Matrix4<S>} 返回4✖4的齐次变换矩阵
   */
  Matrix4<S> getTransForm(const std::string &start, const std::string &end);

  /**
   * @brief:  添加一个变换关系，通过齐次矩阵
   * @param {const std::string&} parent  父节点名字
   * @param {const std::string&} child  子节点名字
   * @param {Matrix4<S>} 父节点到子节点之间的变换关系
   * @return {Matrix4<S>} 返回4✖4的齐次变换矩阵
   */
  Matrix4<S> pushTransForm(const std::string &parent, const std::string &child,
                           Matrix4<S> matrix);

  /**
   * @brief:  添加一个变换关系，通过位置和欧拉角
   * @param {const std::string&} parent  父节点名字
   * @param {const std::string&} child  子节点名字
   * @param {S&} x 沿x轴的平移
   * @param {S&} y 沿y轴的平移
   * @param {S&} z 沿z轴的平移
   * @param {S&} rx 绕x轴的旋转
   * @param {S&} ry 绕y轴的旋转
   * @param {S&} rz 绕z轴的旋转
   * @return {Matrix4<S>} 返回4✖4的齐次变换矩阵
   */
  Matrix4<S> pushTransForm(const std::string &parent, const std::string &child,
                           S &x, S &y, S &z, S &rx, S &ry, S &rz);

  /**
   * @brief:  添加一个变换关系,通过位置和四元数
   * @param {const std::string&} parent  父节点名字
   * @param {const std::string&} child  子节点名字
   * @param {S&} x 沿x轴的平移
   * @param {S&} y 沿y轴的平移
   * @param {S&} z 沿z轴的平移
   * @param {S&} rx 四元数对应的x
   * @param {S&} ry 四元数对应的y
   * @param {S&} 四元数对应的z
   * @param {S&} 四元数对应的w
   * @return {Matrix4<S>} 返回4✖4的齐次变换矩阵
   */
  Matrix4<S> pushTransForm(const std::string &parent, const std::string &child,
                           S &x, S &y, S &z, S &rx, S &ry, S &rz, S &rw);
};

// typedef TransFormsGroup<float> TransFormsGroupf;
// typedef TransFormsGroup<double> TransFormsGroupd;

using TransFormsGroupf = TransFormsGroup<float>;
using TransFormsGroupd = TransFormsGroup<double>;

}  // namespace transforms3d

#include "./trans_forms-inl.h"
#include "./trans_forms_group-inl.h"
#endif
