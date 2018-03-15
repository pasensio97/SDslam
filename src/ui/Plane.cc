/**
 *
 *  Copyright (C) 2018 Eduardo Perdices <eperdices at gsyc dot es>
 *
 *  The following code is a derivative work of the code from the ORB-SLAM2 project,
 *  which is licensed under the GNU Public License, version 3. This code therefore
 *  is also licensed under the terms of the GNU Public License, version 3.
 *  For more information see <https://github.com/raulmur/ORB_SLAM2>.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "Plane.h"

using std::vector;
using std::mutex;
using std::unique_lock;

namespace SD_SLAM {

const float eps = 1e-4;

cv::Mat ExpSO3(const float &x, const float &y, const float &z) {
  cv::Mat I = cv::Mat::eye(3,3,CV_32F);
  const float d2 = x*x+y*y+z*z;
  const float d = sqrt(d2);
  cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
               z, 0, -x,
               -y,  x, 0);
  if(d<eps)
    return (I + W + 0.5f*W*W);
  else
    return (I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2);
}

cv::Mat ExpSO3(const cv::Mat &v) {
  return ExpSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

Plane::Plane(const std::vector<MapPoint *> &vMPs, const Eigen::Matrix4d &pose): mvMPs(vMPs), mPose(pose) {
  rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
  Recompute();
}

void Plane::Recompute() {
  const int N = mvMPs.size();

  // Recompute plane with all points
  cv::Mat A = cv::Mat(N,4,CV_32F);
  A.col(3) = cv::Mat::ones(N,1,CV_32F);

  o.setZero();

  int nPoints = 0;
  for(int i=0; i<N; i++) {
    MapPoint* pMP = mvMPs[i];
    if(!pMP->isBad()) {
      Eigen::Vector3d Xw = pMP->GetWorldPos();
      o += Xw;

      A.at<float>(nPoints, 0) = Xw(0);
      A.at<float>(nPoints, 1) = Xw(1);
      A.at<float>(nPoints, 2) = Xw(2);

      nPoints++;
    }
  }
  A.resize(nPoints);

  cv::Mat u,w,vt;
  cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

  float a = vt.at<float>(3,0);
  float b = vt.at<float>(3,1);
  float c = vt.at<float>(3,2);

  o = o*(1.0f/nPoints);
  const float f = 1.0f/sqrt(a*a+b*b+c*c);

  // Compute XC just the first time
  if(XC.isZero(0)) {
      Eigen::Vector3d Oc = -mPose.block<3, 3>(0, 0).transpose() * mPose.block<3, 1>(0, 3);
      XC = Oc-o;
  }

  if((XC(0)*a+XC(1)*b+XC(2)*c)<0) {
      a=-a;
      b=-b;
      c=-c;
  }

  const float nx = a*f;
  const float ny = b*f;
  const float nz = c*f;

  n = (cv::Mat_<float>(3,1)<<nx,ny,nz);

  cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

  cv::Mat v = up.cross(n);
  const float sa = cv::norm(v);
  const float ca = up.dot(n);
  const float ang = atan2(sa,ca);
  Tpw = cv::Mat::eye(4,4,CV_32F);

  Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*ang/sa)*ExpSO3(up*rang);
  Tpw.at<float>(0,3) = o(0);
  Tpw.at<float>(1,3) = o(1);
  Tpw.at<float>(2,3) = o(2);

  glTpw.m[0] = Tpw.at<float>(0,0);
  glTpw.m[1] = Tpw.at<float>(1,0);
  glTpw.m[2] = Tpw.at<float>(2,0);
  glTpw.m[3]  = 0.0;

  glTpw.m[4] = Tpw.at<float>(0,1);
  glTpw.m[5] = Tpw.at<float>(1,1);
  glTpw.m[6] = Tpw.at<float>(2,1);
  glTpw.m[7]  = 0.0;

  glTpw.m[8] = Tpw.at<float>(0,2);
  glTpw.m[9] = Tpw.at<float>(1,2);
  glTpw.m[10] = Tpw.at<float>(2,2);
  glTpw.m[11]  = 0.0;

  glTpw.m[12] = Tpw.at<float>(0,3);
  glTpw.m[13] = Tpw.at<float>(1,3);
  glTpw.m[14] = Tpw.at<float>(2,3);
  glTpw.m[15]  = 1.0;
}

}  // namespace SD_SLAM
