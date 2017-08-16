/**
 *
 *  Copyright (C) 2017 Eduardo Perdices <eperdices at gsyc dot es>
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

#ifndef ORB_SLAM2_IMAGEALIGN_H_
#define ORB_SLAM2_IMAGEALIGN_H_

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "Frame.h"
/*#include "extra/utils.h"*/

namespace ORB_SLAM2 {

class ImageAlign {
 public:
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImageAlign();
  ~ImageAlign();

  // Compute Pose between frames
  bool ComputePose(Frame &CurrentFrame, const Frame &LastFrame, bool fast = false);

  //inline double GetError() { return error_; }

 private:
  // Optimize using Gauss Newton strategy
  void Optimize(const Frame &CurrentFrame, const Frame &LastFrame, cv::Mat &se3, int level);

  // Compute residual and jacobians
  double ComputeResiduals(const Frame &CurrentFrame, const Frame &LastFrame, const cv::Mat &se3,
                          int level, bool linearize, bool patches);

  // Compute patches within a pyramid level
  void PrecomputePatches(const Frame &LastFrame, int level);

  // Project point in image
  bool Project(const Frame &frame, const cv::Mat &se3, MapPoint *point, Eigen::Vector2d &res);

  // Jacobian of 3D point projection in frame coordinates to unit plane coordinates
  void Jacobian3DToPlane(const Eigen::Vector3d &p, Eigen::Matrix<double, 2, 6> *J);

  // Get absolute max value of a vector
  double AbsMax(const Eigen::VectorXd &v);

  // Exponential vector
  cv::Mat Exp(const Eigen::Matrix<double, 6, 1> &update);
  Eigen::Quaterniond RotationExp(const Eigen::Vector3d &omega, double *theta);
  Eigen::Matrix3d RotationHat(const Eigen::Vector3d &v);

  int patch_size_;    // Patch size
  int min_level_;     // Min search level
  int max_level_;     // Max search level
  int max_its_;       // Max align iterations

  double chi2_;
  size_t  n_meas_;    // Number of measurements
  bool stop_;         // Stop flag
  double error_;      // Last optimization error

  cv::Mat patch_cache_;                 // Cache for patches
  std::vector<bool> visible_fts_;       // Visible points
  Eigen::Matrix<double, 6, 6>  H_;      // Hessian approximation
  Eigen::Matrix<double, 6, 1>  Jres_;   // Store Jacobian residual
  Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::ColMajor> jacobian_cache_;
};

}  // namespace ORB_SLAM2

#endif  // ORB_SLAM2_IMAGEALIGN_H_
