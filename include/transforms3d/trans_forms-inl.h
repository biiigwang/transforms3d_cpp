/*
 * @Descripttion: cpp实现
 * @Author: sangxin
 * @Date: 2021-05-01 21:04:19
 * @LastEditTime: 2021-05-02 23:39:20
 */
#ifndef TRANS_FORMS_INL_H
#define TRANS_FORMS_INL_H

#include <transforms3d/transforms3d.h>

namespace transforms3d {

extern template class TransForms<double>;
extern template class TransForms<float>;

/*---------------------------------------角度弧度转换----------------------------------------*/
/**
 * @brief: 角度转为弧度
 * @param {S} angle 角度值
 * @return 返回对应弧度值,一般在-3.14~3.14之间
 */
template <typename S>
S TransForms<S>::Degrees(S angle) {
  return angle / 180 * M_PI;
}

/**
 * @brief: 弧度转为角度
 * @param {S} degrees 弧度值
 * @return 返回对应的角度值，一般在-180~180之间
 */
template <typename S>
S TransForms<S>::Angle(S degrees) {
  return degrees / M_PI * 180;
}

/*---------------------------------------欧拉角部分---------------------------*/
/**
 * @brief: 角度制欧拉角转旋转矩阵,此函数默认的旋转顺序是x-y-z.
 * @param {S} rx 绕x轴的旋转.
 * @param {S} ry 绕y轴的旋转.
 * @param {S} rz 绕z轴的旋转.
 * @return {Matrix3<S>}  返回3✖3的旋转矩阵.
 */
template <typename S>
Matrix3<S> TransForms<S>::EulerAngle2Mat(S rx, S ry, S rz) {
  rx = Degrees(rx);
  ry = Degrees(ry);
  rz = Degrees(rz);
  AngleAxis<S> rollAngle(AngleAxis<S>(rx, Vector3<S>::UnitX()));
  AngleAxis<S> pitchAngle(AngleAxis<S>(ry, Vector3<S>::UnitY()));
  AngleAxis<S> yawAngle(AngleAxis<S>(rz, Vector3<S>::UnitZ()));
  Matrix3<S> rotation_matrix;
  rotation_matrix = yawAngle * pitchAngle * rollAngle;
  return rotation_matrix;
}
/**
 * @brief: 欧拉角转旋转矩阵
 * @param {Vector3<S>} eular 欧拉角rx,ry,rz
 * @return {Matrix3<S>} 返回3✖3的旋转矩阵.
 */
template <typename S>
Matrix3<S> TransForms<S>::Euler2Mat(S rx, S ry, S rz) {
  AngleAxis<S> rollAngle(AngleAxis<S>(rx, Vector3<S>::UnitX()));
  AngleAxis<S> pitchAngle(AngleAxis<S>(ry, Vector3<S>::UnitY()));
  AngleAxis<S> yawAngle(AngleAxis<S>(rz, Vector3<S>::UnitZ()));
  Matrix3<S> rotation_matrix;
  rotation_matrix = yawAngle * pitchAngle * rollAngle;
  return rotation_matrix;
}

/**
 * @brief:欧拉角转四元数
 * @param {S} rx 绕x轴的旋转
 * @param {S} ry 绕y轴的旋转
 * @param {S} rz 绕z轴的旋转
 * @return {Quaternion<S>} 返回对应的四元数
 */
template <typename S>
Quaternion<S> TransForms<S>::Euler2Quat(S rx, S ry, S rz) {
  return Eigen::AngleAxis<S>(rx, ::Eigen::Vector3<S>::UnitX()) *
         Eigen::AngleAxis<S>(ry, ::Eigen::Vector3<S>::UnitY()) *
         Eigen::AngleAxis<S>(rz, ::Eigen::Vector3<S>::UnitZ());
}

/**
 * @brief: 角度制欧拉角转四元数
 * @param {S} rx 绕x轴的旋转
 * @param {S} ry 绕y轴的旋转
 * @param {S} rz 绕z轴的旋转
 * @return {Quaternion<S>} 返回对应的四元数
 */
template <typename S>
Quaternion<S> TransForms<S>::EulerAngle2Quat(S rx, S ry, S rz) {
  rx = Degrees(rx);
  ry = Degrees(ry);
  rz = Degrees(rz);
  return Eigen::AngleAxis<S>(rx, ::Eigen::Vector3<S>::UnitX()) *
         Eigen::AngleAxis<S>(ry, ::Eigen::Vector3<S>::UnitY()) *
         Eigen::AngleAxis<S>(rz, ::Eigen::Vector3<S>::UnitZ());
}
/**
 * @brief: 旋转矩阵转欧拉角(弧度制)
 * @param {Matrix3<S>} 3✖3的旋转矩阵
 * @return {Vector3<S>} 欧拉角
 */
template <typename S>
Vector3<S> TransForms<S>::Mat2Euler(Matrix3<S> mat) {
  return mat.eulerAngles(0, 1, 2);
}

/**
 * @brief: 欧拉角转旋转矩阵
 * @param {S} rx 绕x轴的旋转
 * @param {S} ry 绕y轴的旋转
 * @param {S} rz 绕z轴的旋转
 * @return {*}
 */
template <typename S>
Matrix3<S> TransForms<S>::EulerAngle2Mat(Vector3<S> eular) {
  return EulerAngle2Mat(eular.x(), eular.y(), eular.z());
}

/**
 * @brief: 旋转矩阵转角度制欧拉角(角度制)
 * @param {Matrix3<S>} 3✖3的旋转矩阵
 * @return {Vector3<S>}  欧拉角
 */
template <typename S>
Vector3<S> TransForms<S>::Mat2EulerAngle(Matrix3<S> mat) {
  Vector3<S> rot = mat.eulerAngles(2, 1, 0);
  rot = rot / M_PI * 180;
  return rot;
}

/*---------------------------------------四元数部分----------------------------------------*/
/**
 * @brief: 四元数转旋转矩阵
 * @param {Quaternion<S>} 四元数
 * @return {Matrix3<S>} 对应的旋转矩阵
 */
template <typename S>
Matrix3<S> TransForms<S>::Quat2Mat(Quaternion<S> quat) {
  return quat.matrix();
}

/**
 * @brief: 四元数转欧拉角
 * @param {Quaternion<S>} 四元数
 * @return {Vector3<S>} 对应的欧拉角
 */
template <typename S>
Vector3<S> TransForms<S>::Quat2Eular(Quaternion<S> quat) {
  return Mat2Euler(quat.matrix());
}

/**
 * @brief: 四元数转弧度制欧拉角(角度制)
 * @param {Quaternion<S>} 四元数
 * @return {Vector3<S>} 对应的欧拉角
 */
template <typename S>
Vector3<S> TransForms<S>::Quat2EularAngle(Quaternion<S> quat) {
  return Mat2EulerAngle(quat.matrix());
}

/**
 * @brief: 旋转矩阵转四元数
 * @param {Matrix3<S>} 3✖3的旋转矩阵
 * @return {Quaternion<S>} 对应的四元数
 */
template <typename S>
Quaternion<S> TransForms<S>::Mat2Quat(Matrix3<S> mat) {
  return Quaternion<S>(mat);
}

/*---------------------------------------齐次矩阵部分----------------------------------------*/
/**
 * @brief: 通过位置和欧拉角合成一个齐次矩阵
 * @param {Vector3<S>} positon 平移位置
 * @param {Vector3<S>} rotEular  旋转变换(欧拉角形式)
 * @return {*}
 */
template <typename S>
Matrix4<S> TransForms<S>::Compose(Vector3<S> positon, Vector3<S> rotEular) {
  Matrix3<S> rot = TransForms<S>::EulerAngle2Mat(rotEular);
  // std::cout<<Mat2EulerAngle(rot);
  Matrix4<S> t;
  t.setIdentity();
  t.template block<3, 3>(0, 0) = rot;
  t.template block<3, 1>(0, 3) = positon;
  return t;
}

/**
 * @brief: 通过位置和四元数合成一个齐次矩阵
 * @param {Vector3<S>} positon 平移位置
 * @param {Quaternion<S>} quat 四元数
 * @return {Matrix4<S>} 齐次矩阵
 */
template <typename S>
Matrix4<S> TransForms<S>::Compose(Vector3<S> positon, Quaternion<S> quat) {
  return Compose(positon, Quat2Eular(quat));
}

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
template <typename S>
Matrix4<S> TransForms<S>::ComposeEuler(const S x, const S y, const S z,
                                       const S rx, const S ry, const S rz) {
  Eigen::Vector3<S> rot(rx, ry, rz);
  Eigen::Vector3<S> pos(x, y, z);
  return TransForms<S>::Compose(pos, rot);
}

/**
 * @brief:  将齐次矩阵转换成平移和欧拉角形式，方便理解
 * @param {Matrix4<S>} 4✖4的齐次变换矩阵
 * @return {VectorX<S>} x,y,z,rx,ry,rz
 */
template <typename S>
VectorX<S> TransForms<S>::H2EulerAngle(Matrix4<S> t) {
  VectorX<S> pose = VectorX<S>(6);
  Matrix3<S> mt = t.template block<3, 3>(0, 0);
  Vector3<S> p3 = t.template block<3, 1>(0, 3).col(0);
  pose(0, 0) = p3.x();
  pose(1, 0) = p3.y();
  pose(2, 0) = p3.z();
  Vector3<S> eular = Mat2EulerAngle(mt);
  pose(3, 0) = eular.x();
  pose(4, 0) = eular.y();
  pose(5, 0) = eular.z();
  return pose;
}

/**
 * @brief:  将齐次矩阵转换成平移和旋转矩阵形式
 * @param {Matrix4<S>} 4✖4的齐次变换矩阵
 * @return {VectorX<S>} x,y,z,rx,ry,rz
 */

template <typename S>
Matrix3<S> TransForms<S>::HDecompose(Matrix4<S> t, Matrix3<S> &rotate,
                                     Vector3<S> &position) {
  rotate = t.template block<3, 3>(0, 0);
  position = t.template block<3, 1>(0, 3).col(0);
  return rotate;
}
}  // namespace transforms3d
#endif  // TRANS_FORMS_INL_H
