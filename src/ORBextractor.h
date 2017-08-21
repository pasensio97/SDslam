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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>

namespace SD_SLAM {

class ORBextractor {
 public:
  enum {HARRIS_SCORE=0, FAST_SCORE=1 };

  ORBextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST);

  ~ORBextractor(){}

  // Compute the ORB features and descriptors on an image.
  // ORB are dispersed on the image using an octree.
  // Mask is ignored in the current implementation.
  void operator()(cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints,
                  cv::OutputArray descriptors, std::vector<cv::Mat> &imagePyramid);

  int inline GetLevels() {
    return nlevels;
  }

  float inline GetScaleFactor() {
    return scaleFactor;
  }

  std::vector<float> inline GetScaleFactors() {
    return mvScaleFactor;
  }

  std::vector<float> inline GetInverseScaleFactors() {
    return mvInvScaleFactor;
  }

  std::vector<float> inline GetScaleSigmaSquares() {
    return mvLevelSigma2;
  }

  std::vector<float> inline GetInverseScaleSigmaSquares() {
    return mvInvLevelSigma2;
  }

 protected:
  void ComputePyramid(cv::Mat image, std::vector<cv::Mat> &imagePyramid);
  void ComputeKeyPoints(std::vector<std::vector<cv::KeyPoint> >& allKeypoints, std::vector<cv::Mat> &imagePyramid);
  std::vector<cv::Point> pattern;

  int nfeatures;
  double scaleFactor;
  int nlevels;
  int iniThFAST;
  int minThFAST;

  std::vector<int> mnFeaturesPerLevel;

  std::vector<int> umax;

  std::vector<float> mvScaleFactor;
  std::vector<float> mvInvScaleFactor;  
  std::vector<float> mvLevelSigma2;
  std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif

