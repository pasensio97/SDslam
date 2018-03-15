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

#ifndef SD_SLAM_PLANE_H
#define SD_SLAM_PLANE_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pangolin/pangolin.h>
#include <Eigen/Dense>
#include "MapPoint.h"

namespace SD_SLAM {

class Plane {
 public:
  Plane(const std::vector<MapPoint*> &vMPs, const Eigen::Matrix4d &pose);

  void Recompute();

  //normal
  cv::Mat n;
  //origin
  Eigen::Vector3d o;
  //arbitrary orientation along normal
  float rang;
  //transformation from world to the plane
  cv::Mat Tpw;
  pangolin::OpenGlMatrix glTpw;
  //MapPoints that define the plane
  std::vector<MapPoint*> mvMPs;
  //camera pose when the plane was first observed (to compute normal direction)
  Eigen::Matrix4d mPose;
  Eigen::Vector3d XC;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_PLANE_H
