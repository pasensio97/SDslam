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

#ifndef SD_SLAM_INITIALIZER_H
#define SD_SLAM_INITIALIZER_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "Frame.h"

namespace SD_SLAM {

// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE RGBD CASE.
class Initializer {
  typedef std::pair<int, int> Match;

 public:
  // Fix the reference frame
  Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

  // Computes in parallel a fundamental matrix and a homography
  // Selects a model and tries to recover the motion and the structure from motion
  bool Initialize(const Frame &CurrentFrame, const std::vector<int> &vMatches12,
          Eigen::Matrix3d &R21, Eigen::Vector3d &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated);

 private:
  void FindHomography(std::vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
  void FindFundamental(std::vector<bool> &vbInliers, float &score, cv::Mat &F21);

  cv::Mat ComputeH21(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);
  cv::Mat ComputeF21(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);

  float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, std::vector<bool> &vbMatchesInliers, float sigma);

  float CheckFundamental(const cv::Mat &F21, std::vector<bool> &vbMatchesInliers, float sigma);

  bool ReconstructF(std::vector<bool> &vbMatchesInliers, cv::Mat &F21, const Eigen::Matrix3d &K,
            Eigen::Matrix3d &R21, Eigen::Vector3d &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

  bool ReconstructH(std::vector<bool> &vbMatchesInliers, cv::Mat &H21, const Eigen::Matrix3d &K,
            Eigen::Matrix3d &R21, Eigen::Vector3d &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

  void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const Eigen::Matrix<double, 3, 4> &P1, const Eigen::Matrix<double, 3, 4> &P2, Eigen::Vector3d &x3D);

  void Normalize(const std::vector<cv::KeyPoint> &vKeys, std::vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

  int CheckRT(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, const std::vector<cv::KeyPoint> &vKeys1,
              const std::vector<cv::KeyPoint> &vKeys2, const std::vector<Match> &vMatches12, std::vector<bool> &vbInliers,
              const Eigen::Matrix3d &K, std::vector<cv::Point3f> &vP3D, float th2, std::vector<bool> &vbGood, float &parallax);

  void DecomposeE(const cv::Mat &E, Eigen::Matrix3d &R1, Eigen::Matrix3d &R2, Eigen::Vector3d &t);


  // Keypoints from Reference Frame (Frame 1)
  std::vector<cv::KeyPoint> mvKeys1;

  // Keypoints from Current Frame (Frame 2)
  std::vector<cv::KeyPoint> mvKeys2;

  // Current Matches from Reference to Current
  std::vector<Match> mvMatches12;
  std::vector<bool> mvbMatched1;

  // Calibration
  Eigen::Matrix3d mK;

  // Standard Deviation and Variance
  float mSigma, mSigma2;

  // Ransac max iterations
  int mMaxIterations;

  // Ransac sets
  std::vector<std::vector<size_t> > mvSets;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_INITIALIZER_H
