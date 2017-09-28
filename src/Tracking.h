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

#ifndef SD_SLAM_TRACKING_H
#define SD_SLAM_TRACKING_H

#include <mutex>
#include <string>
#include <list>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "System.h"
#ifdef PANGOLIN
#include "ui/Viewer.h"
#include "ui/FrameDrawer.h"
#include "ui/MapDrawer.h"
#endif

namespace SD_SLAM {

#ifdef PANGOLIN
class Viewer;
class FrameDrawer;
#endif
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking {
 public:
  Tracking(System* pSys, Map* pMap, const int sensor);

  // Preprocess the input and call Track(). Extract features and performs stereo matching.
  Eigen::Matrix4d GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp);
  Eigen::Matrix4d GrabImageMonocular(const cv::Mat &im, const double &timestamp);

  inline void SetLocalMapper(LocalMapping* pLocalMapper) {
    mpLocalMapper = pLocalMapper;
  }

  inline void SetLoopClosing(LoopClosing* pLoopClosing) {
    mpLoopClosing = pLoopClosing;
  }

#ifdef PANGOLIN
  inline void SetViewer(Viewer* pViewer) {
    mpViewer = pViewer;
  }

  inline void SetFrameDrawer(FrameDrawer* pFrameDrawer) {
    mpFrameDrawer = pFrameDrawer;
  }

  inline void SetMapDrawer(MapDrawer* pMapDrawer) {
    mpMapDrawer = pMapDrawer;
  }
#endif

 public:
  // Tracking states
  enum eTrackingState{
    SYSTEM_NOT_READY = -1,
    NO_IMAGES_YET = 0,
    NOT_INITIALIZED = 1,
    OK = 2,
    LOST = 3
  };

  eTrackingState mState;
  eTrackingState mLastProcessedState;

  // Input sensor
  int mSensor;

  // Current Frame
  Frame mCurrentFrame;
  cv::Mat mImGray;

  // Initialization Variables (Monocular)
  std::vector<int> mvIniLastMatches;
  std::vector<int> mvIniMatches;
  std::vector<cv::Point2f> mvbPrevMatched;
  std::vector<cv::Point3f> mvIniP3D;
  Frame mInitialFrame;

  // Lists used to recover the full camera trajectory at the end of the execution.
  // Basically we store the reference keyframe for each frame and its relative transformation
  std::list<Eigen::Matrix4d> mlRelativeFramePoses;
  std::list<KeyFrame*> mlpReferences;
  std::list<double> mlFrameTimes;
  std::list<bool> mlbLost;

  void Reset();

 protected:
  // Main tracking function. It is independent of the input sensor.
  void Track();

  // Map initialization for stereo and RGB-D
  void StereoInitialization();

  // Map initialization for monocular
  void MonocularInitialization();
  void CreateInitialMapMonocular();

  void CheckReplacedInLastFrame();
  bool TrackReferenceKeyFrame();
  void UpdateLastFrame();
  bool TrackWithMotionModel();

  bool Relocalization();

  void UpdateLocalMap();
  void UpdateLocalPoints();
  void UpdateLocalKeyFrames();

  bool TrackLocalMap();
  void SearchLocalPoints();

  bool NeedNewKeyFrame();
  void CreateNewKeyFrame();

  // Other Thread Pointers
  LocalMapping* mpLocalMapper;
  LoopClosing* mpLoopClosing;

  // ORB
  ORBextractor* mpORBextractorLeft;
  ORBextractor* mpIniORBextractor;

  // Initalization (only for monocular)
  Initializer* mpInitializer;

  // Local Map
  KeyFrame* mpReferenceKF;
  std::vector<KeyFrame*> mvpLocalKeyFrames;
  std::vector<MapPoint*> mvpLocalMapPoints;

  // System
  System* mpSystem;

#ifdef PANGOLIN
  // Drawers
  Viewer* mpViewer;
  FrameDrawer* mpFrameDrawer;
  MapDrawer* mpMapDrawer;
#endif

  // Map
  Map* mpMap;

  // Calibration matrix
  Eigen::Matrix3d mK;
  cv::Mat mDistCoef;
  float mbf;

  // New KeyFrame rules (according to fps)
  int mMinFrames;
  int mMaxFrames;

  // Threshold close/far points
  // Points seen as close by the stereo/RGBD sensor are considered reliable
  // and inserted from just one frame. Far points requiere a match in two keyframes.
  float mThDepth;

  // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
  float mDepthMapFactor;

  // Current matches in frame
  int mnMatchesInliers;

  // Last Frame, KeyFrame and Relocalisation Info
  KeyFrame* mpLastKeyFrame;
  Frame mLastFrame;
  unsigned int mnLastKeyFrameId;
  unsigned int mnLastRelocFrameId;

  // Motion Model
  Eigen::Matrix4d mVelocity;

  std::list<MapPoint*> mlpTemporalPoints;
  int threshold_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_TRACKING_H
