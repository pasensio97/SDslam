/**
 *
 *  Copyright (C) 2017 Eduardo Perdices <eperdices at gsyc dot es>
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

#include "Converter.h"

namespace SD_SLAM {

std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors) {
  std::vector<cv::Mat> vDesc;
  vDesc.reserve(Descriptors.rows);
  for (int j = 0; j < Descriptors.rows; j++)
    vDesc.push_back(Descriptors.row(j));

  return vDesc;
}

g2o::SE3Quat Converter::toSE3Quat(const Eigen::Matrix<double, 4, 4> &SE3) {
  Eigen::Matrix<double, 3, 3> R = SE3.block<3, 3>(0, 0);
  Eigen::Matrix<double, 3, 1> t = SE3.block<3, 1>(0, 3);
  return g2o::SE3Quat(R, t);
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 4, 4> &m) {
  cv::Mat cvMat(4, 4, CV_32F);
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      cvMat.at<float>(i, j) = m(i, j);

  return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m) {
  cv::Mat cvMat(3, 3, CV_32F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      cvMat.at<float>(i, j) = m(i, j);

  return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 3, 1> &m) {
  cv::Mat cvMat(3, 1, CV_32F);
  for (int i = 0; i < 3; i++)
      cvMat.at<float>(i) = m(i);

  return cvMat.clone();
}

Eigen::Matrix<double, 3, 1> Converter::toVector3d(const cv::Mat &cvVector) {
  Eigen::Matrix<double, 3, 1> v;
  v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

  return v;
}

Eigen::Matrix<double, 3, 1> Converter::toVector3d(const cv::Point3f &cvPoint) {
  Eigen::Matrix<double, 3, 1> v;
  v << cvPoint.x, cvPoint.y, cvPoint.z;

  return v;
}

Eigen::Matrix<double, 3, 3> Converter::toMatrix3d(const cv::Mat &cvMat3) {
  Eigen::Matrix<double, 3, 3> M;

  M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
     cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
     cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);

  return M;
}

Eigen::Matrix<double, 4, 4> Converter::toMatrix4d(const cv::Mat &cvMat4) {
  Eigen::Matrix<double, 4, 4> M;

  M << cvMat4.at<float>(0, 0), cvMat4.at<float>(0, 1), cvMat4.at<float>(0, 2), cvMat4.at<float>(0, 3),
     cvMat4.at<float>(1, 0), cvMat4.at<float>(1, 1), cvMat4.at<float>(1, 2), cvMat4.at<float>(1, 3),
     cvMat4.at<float>(2, 0), cvMat4.at<float>(2, 1), cvMat4.at<float>(2, 2), cvMat4.at<float>(2, 3),
     cvMat4.at<float>(3, 0), cvMat4.at<float>(3, 1), cvMat4.at<float>(3, 2), cvMat4.at<float>(3, 3);

  return M;
}

Eigen::Matrix<double, 4, 4> Converter::toMatrix4d(const g2o::SE3Quat &SE3) {
  return SE3.to_homogeneous_matrix();
}

Eigen::Matrix<double, 4, 4> Converter::toMatrix4d(const g2o::Sim3 &Sim3) {
  Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
  Eigen::Vector3d eigt = Sim3.translation();
  double s = Sim3.scale();
  return toSE3(s*eigR, eigt);
}

Eigen::Matrix<double, 4, 4> Converter::toSE3(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 3, 1> &t) {
  Eigen::Matrix4d mat;

  mat.setIdentity();
  mat.block<3, 3>(0, 0) = R;
  mat.block<3, 1>(0, 3) = t;

  return mat;
}

Eigen::Matrix<double, 4, 4> Converter::inverted_pose(const Eigen::Vector3d &position, const Eigen::Matrix<double, 3, 3> &orientation){
  Eigen::Matrix<double, 4, 4> RTc = Eigen::Matrix<double, 4, 4>::Identity();  
  RTc.block<3, 3>(0, 0) = orientation.inverse();
  RTc.block<3, 1>(0, 3) = -RTc.block<3, 3>(0, 0) * position;
  return RTc;
}

Eigen::Matrix<double, 4, 4> Converter::inverted_pose(const Eigen::Matrix<double, 4, 4> &world_pose){
  return Converter::inverted_pose(world_pose.block<3,1>(0,3), world_pose.block<3,3>(0,0));
}

Eigen::Matrix<double, 4, 4> Converter::inverted_pose(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation){
  return Converter::inverted_pose(position.block<3,1>(0,3), orientation.toRotationMatrix().normalized());
}


}  // namespace SD_SLAM
