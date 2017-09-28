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

#ifndef SD_SLAM_SIM3SOLVER_H
#define SD_SLAM_SIM3SOLVER_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "KeyFrame.h"

namespace SD_SLAM {

class Sim3Solver {
 public:
  Sim3Solver(KeyFrame* pKF1, KeyFrame* pKF2, const std::vector<MapPoint*> &vpMatched12, const bool bFixScale = true);

  void SetRansacParameters(double probability = 0.99, int minInliers = 6 , int maxIterations = 300);

  Eigen::Matrix4d find(std::vector<bool> &vbInliers12, int &nInliers);

  Eigen::Matrix4d iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);

  Eigen::Matrix3d GetEstimatedRotation();
  Eigen::Vector3d GetEstimatedTranslation();
  float GetEstimatedScale();


 protected:
  void ComputeCentroid(const Eigen::Matrix3d &P, Eigen::Matrix3d &Pr, Eigen::Vector3d &C);

  void ComputeSim3(const Eigen::Matrix3d &P1, const Eigen::Matrix3d &P2);

  void CheckInliers();

  void Project(const std::vector<Eigen::Vector3d> &vP3Dw, std::vector<Eigen::Vector2d> &vP2D, const Eigen::Matrix4d &Tcw, const Eigen::Matrix3d &K);
  void FromCameraToImage(const std::vector<Eigen::Vector3d> &vP3Dc, std::vector<Eigen::Vector2d> &vP2D, const Eigen::Matrix3d &K);


 protected:
  // KeyFrames and matches
  KeyFrame* mpKF1;
  KeyFrame* mpKF2;

  std::vector<Eigen::Vector3d> mvX3Dc1;
  std::vector<Eigen::Vector3d> mvX3Dc2;
  std::vector<MapPoint*> mvpMapPoints1;
  std::vector<MapPoint*> mvpMapPoints2;
  std::vector<MapPoint*> mvpMatches12;
  std::vector<size_t> mvnIndices1;
  std::vector<size_t> mvSigmaSquare1;
  std::vector<size_t> mvSigmaSquare2;
  std::vector<size_t> mvnMaxError1;
  std::vector<size_t> mvnMaxError2;

  int N;
  int mN1;

  // Current Estimation
  Eigen::Matrix3d mR12i;
  Eigen::Vector3d mt12i;
  float ms12i;
  Eigen::Matrix4d mT12i;
  Eigen::Matrix4d mT21i;
  std::vector<bool> mvbInliersi;
  int mnInliersi;

  // Current Ransac State
  int mnIterations;
  std::vector<bool> mvbBestInliers;
  int mnBestInliers;
  Eigen::Matrix4d mBestT12;
  Eigen::Matrix3d mBestRotation;
  Eigen::Vector3d mBestTranslation;
  float mBestScale;

  // Scale is fixed to 1 in the stereo/RGBD case
  bool mbFixScale;

  // Indices for random selection
  std::vector<size_t> mvAllIndices;

  // Projections
  std::vector<Eigen::Vector2d> mvP1im1;
  std::vector<Eigen::Vector2d> mvP2im2;

  // RANSAC probability
  double mRansacProb;

  // RANSAC min inliers
  int mRansacMinInliers;

  // RANSAC max iterations
  int mRansacMaxIts;

  // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
  float mTh;
  float mSigma2;

  // Calibration
  Eigen::Matrix3d mK1;
  Eigen::Matrix3d mK2;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_SIM3SOLVER_H
