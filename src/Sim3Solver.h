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
#include "KeyFrame.h"

namespace SD_SLAM {

class Sim3Solver {
 public:
  Sim3Solver(KeyFrame* pKF1, KeyFrame* pKF2, const std::vector<MapPoint*> &vpMatched12, const bool bFixScale = true);

  void SetRansacParameters(double probability = 0.99, int minInliers = 6 , int maxIterations = 300);

  cv::Mat find(std::vector<bool> &vbInliers12, int &nInliers);

  cv::Mat iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);

  cv::Mat GetEstimatedRotation();
  cv::Mat GetEstimatedTranslation();
  float GetEstimatedScale();


 protected:
  void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);

  void ComputeSim3(cv::Mat &P1, cv::Mat &P2);

  void CheckInliers();

  void Project(const std::vector<cv::Mat> &vP3Dw, std::vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K);
  void FromCameraToImage(const std::vector<cv::Mat> &vP3Dc, std::vector<cv::Mat> &vP2D, cv::Mat K);


 protected:
  // KeyFrames and matches
  KeyFrame* mpKF1;
  KeyFrame* mpKF2;

  std::vector<cv::Mat> mvX3Dc1;
  std::vector<cv::Mat> mvX3Dc2;
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
  cv::Mat mR12i;
  cv::Mat mt12i;
  float ms12i;
  cv::Mat mT12i;
  cv::Mat mT21i;
  std::vector<bool> mvbInliersi;
  int mnInliersi;

  // Current Ransac State
  int mnIterations;
  std::vector<bool> mvbBestInliers;
  int mnBestInliers;
  cv::Mat mBestT12;
  cv::Mat mBestRotation;
  cv::Mat mBestTranslation;
  float mBestScale;

  // Scale is fixed to 1 in the stereo/RGBD case
  bool mbFixScale;

  // Indices for random selection
  std::vector<size_t> mvAllIndices;

  // Projections
  std::vector<cv::Mat> mvP1im1;
  std::vector<cv::Mat> mvP2im2;

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
  cv::Mat mK1;
  cv::Mat mK2;
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_SIM3SOLVER_H
