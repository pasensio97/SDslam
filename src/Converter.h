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

#ifndef SD_SLAM_CONVERTER_H
#define SD_SLAM_CONVERTER_H

#include <opencv2/core/core.hpp>

#include <Eigen/Dense>
#include "g2o/types/types_six_dof_expmap.h"
#include "g2o/types/types_seven_dof_expmap.h"

namespace SD_SLAM {

class Converter {
 public:
  static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

  static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
  static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

  static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
  static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
  static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
  static cv::Mat toCvMat(const Eigen::Matrix3d &m);
  static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
  static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

  static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
  static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
  static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

  static std::vector<float> toQuaternion(const cv::Mat &M);
};

}  //namespace SD_SLAM

#endif // SD_SLAM_CONVERTER_H
