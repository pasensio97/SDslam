/**
 *
 *  Copyright (C) 2017-2018 Eduardo Perdices <eperdices at gsyc dot es>
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

#ifndef SD_SLAM_FRAMEDRAWER_H
#define SD_SLAM_FRAMEDRAWER_H

#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pangolin/pangolin.h>
#include <Eigen/Dense>
#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"
#include "ui/Plane.h"

namespace SD_SLAM {

class Tracking;
class Viewer;

class FrameDrawer {
 public:
  FrameDrawer(Map* pMap);

  // Update info from the last processed frame.
  void Update(const cv::Mat &im, const Eigen::Matrix4d &pose, Tracking *pTracker);

  // Draw last processed frame.
  cv::Mat DrawFrame();

  void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

  // Check created planes
  void CheckPlanes(bool recompute);

  // Configure image distortion
  inline void SetUndistort(bool value) { undistort = value; }

  // Add new plane
  inline void AddPlane() { addPlane = true; }

  // Clear planes
  inline void ClearAR() { clearAR = true; }

 protected:
  void DrawTextInfo(cv::Mat &im, int nState);

  Plane* DetectPlane(const Eigen::Matrix4d &pose, const std::vector<MapPoint*> &vMPs, const int iterations=50);

  void DrawCube(const float &size, const float x=0, const float y=0, const float z=0);
  void DrawPlane(int ndivs, float ndivsize);
  void DrawPlane(Plane* pPlane, int ndivs, float ndivsize);

  // Last processed frame data
  cv::Mat mIm;
  Eigen::Matrix4d mPose;
  int mState;
  std::vector<cv::KeyPoint> mvCurrentKeys;
  std::vector<bool> mvbMap;
  std::vector<MapPoint*> mvMPs;
  bool onlyTracking_;

  int mnTracked;

  // Initial points
  std::vector<cv::KeyPoint> mvIniKeys;
  std::vector<int> mvIniMatches;

  std::vector<Plane*> vpPlane;
  bool addPlane;
  bool clearAR;

  bool undistort;
  Map* mpMap;
  std::mutex mMutex;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_FRAMEDRAWER_H
