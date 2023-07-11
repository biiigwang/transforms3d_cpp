/*
 * @Descripttion:
 * @Author: sangxin
 * @Date: 2021-05-01 21:04:19
 * @LastEditTime: 2021-05-02 23:39:20
 */
#include <cmath>
#include <iostream>
#include <transforms3d/transforms3d.h>
using namespace std;
using namespace Eigen;
using namespace transforms3d;

#include "gtest/gtest.h"

TEST(TestTransFormGroup, BaseFindPath) {
  /* base@grapper  */
  Matrix4d Tbg =
      TransFormsd::ComposeEuler(-0.544, -0.203, -0.037, 180, 0.00000, 140);
  /*  camera@maker*/
  Matrix4d Tcw = TransFormsd::ComposeEuler(0.020, -0.040, 0.300, 0, 0, -45);
  /*  base@bottle  */
  Matrix4d Tbw = TransFormsd::ComposeEuler(-0.663, -0.193, -0.231, -180, 0, 140);

  TransFormsGroupd tfg;
  tfg.pushTransForm("base", "grapper", Tbg);
  tfg.pushTransForm("camera", "bottle", Tcw);
  tfg.pushTransForm("base", "bottle", Tbw);

  cout << tfg.toString() << endl;
  cout << TransFormsd::H2EulerAngle(tfg.getTransForm("grapper", "camera"));
  Matrix4d Tgc = Tbg.inverse() * Tbw * Tcw.inverse();
  cout << TransFormsd::H2EulerAngle(Tgc);
}
