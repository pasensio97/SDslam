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

#ifndef SD_SLAM_KEYFRAME_H
#define SD_SLAM_KEYFRAME_H

#include <map>
#include <mutex>
#include "MapPoint.h"
#include "ORBextractor.h"
#include "Frame.h"

namespace SD_SLAM {

class Map;
class MapPoint;
class Frame;

class KeyFrame {
 public:
  KeyFrame(Frame &F, Map* pMap);

  inline int GetID() const { return mnId; }
  void SetID(int n);

  // Pose functions
  void SetPose(const Eigen::Matrix4d &Tcw);
  Eigen::Matrix4d GetPose();
  Eigen::Matrix4d GetPoseInverse();
  Eigen::Vector3d GetCameraCenter();
  Eigen::Matrix3d GetRotation();
  Eigen::Vector3d GetTranslation();

  // Covisibility graph functions
  void AddConnection(KeyFrame* pKF, const int &weight);
  void EraseConnection(KeyFrame* pKF);
  void UpdateConnections();
  void UpdateBestCovisibles();
  std::set<KeyFrame *> GetConnectedKeyFrames();
  std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
  std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
  std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
  int GetWeight(KeyFrame* pKF);

  // Spanning tree functions
  void AddChild(KeyFrame* pKF);
  void EraseChild(KeyFrame* pKF);
  void ChangeParent(KeyFrame* pKF);
  std::set<KeyFrame*> GetChilds();
  KeyFrame* GetParent();
  bool hasChild(KeyFrame* pKF);

  // Loop Edges
  void AddLoopEdge(KeyFrame* pKF);
  std::set<KeyFrame*> GetLoopEdges();

  // MapPoint observation functions
  void AddMapPoint(MapPoint* pMP, const size_t &idx);
  int AddMapPoint(MapPoint* pMP, const Eigen::Vector2d &pos);
  void EraseMapPointMatch(const size_t &idx);
  void EraseMapPointMatch(MapPoint* pMP);
  void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
  std::set<MapPoint*> GetMapPoints();
  std::vector<MapPoint*> GetMapPointMatches();
  int TrackedMapPoints(const int &minObs);
  MapPoint* GetMapPoint(const size_t &idx);

  // KeyPoint functions
  std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
  Eigen::Vector3d UnprojectStereo(int i);

  // Image
  bool IsInImage(const float &x, const float &y) const;

  // Enable/Disable bad flag changes
  void SetNotErase();
  void SetErase();

  // Set/check bad flag
  void SetBadFlag();
  bool isBad();

  // Compute Scene Depth (q=2 median). Used in monocular.
  float ComputeSceneMedianDepth(const int q);

  static bool weightComp( int a, int b){
    return a>b;
  }

  static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
    return pKF1->mnId<pKF2->mnId;
  }

  // The following variables are accesed from only 1 thread or never change (no mutex needed).
 public:
  static long unsigned int nNextId;
  long unsigned int mnId;

  // Grid (to speed up feature matching)
  const int mnGridCols;
  const int mnGridRows;
  const float mfGridElementWidthInv;
  const float mfGridElementHeightInv;

  // Variables used by the tracking
  long unsigned int mnTrackReferenceForFrame;
  long unsigned int mnFuseTargetForKF;

  // Variables used by the local mapping
  long unsigned int mnBALocalForKF;
  long unsigned int mnBAFixedForKF;

  // Variables used by the keyframe database
  long unsigned int mnLoopQuery;
  int mnLoopWords;
  float mLoopScore;
  long unsigned int mnRelocQuery;
  int mnRelocWords;
  float mRelocScore;

  // Variables used by loop closing
  Eigen::Matrix4d mTcwGBA;
  Eigen::Matrix4d mTcwBefGBA;
  long unsigned int mnBAGlobalForKF;

  // Calibration parameters
  const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

  // Number of KeyPoints
  const int N;

  // KeyPoints, stereo coordinate and descriptors (all associated by an index)
  const std::vector<cv::KeyPoint> mvKeys;
  const std::vector<cv::KeyPoint> mvKeysUn;
  const std::vector<float> mvuRight; // negative value for monocular points
  const std::vector<float> mvDepth; // negative value for monocular points
  const cv::Mat mDescriptors;

  // Scale
  const int mnScaleLevels;
  const float mfScaleFactor;
  const float mfLogScaleFactor;
  const std::vector<float> mvScaleFactors;
  const std::vector<float> mvLevelSigma2;
  const std::vector<float> mvInvLevelSigma2;

  // Image bounds and calibration
  const int mnMinX;
  const int mnMinY;
  const int mnMaxX;
  const int mnMaxY;
  Eigen::Matrix3d mK;

  // Image pyramid
  std::vector<cv::Mat> mvImagePyramid;

  // Source filename (if exists)
  std::string mFilename;

  // The following variables need to be accessed trough a mutex to be thread safe.
 protected:
  // SE3 Pose and camera center
  Eigen::Matrix4d Tcw;
  Eigen::Matrix4d Twc;
  Eigen::Vector3d Ow;

  // MapPoints associated to keypoints
  std::vector<MapPoint*> mvpMapPoints;

  // Grid over the image to speed up feature matching
  std::vector< std::vector <std::vector<size_t> > > mGrid;

  std::map<KeyFrame*, int> mConnectedKeyFrameWeights;
  std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
  std::vector<int> mvOrderedWeights;

  // Spanning Tree and Loop Edges
  bool mbFirstConnection;
  KeyFrame* mpParent;
  std::set<KeyFrame*> mspChildrens;
  std::set<KeyFrame*> mspLoopEdges;

  // Bad flags
  bool mbNotErase;
  bool mbToBeErased;
  bool mbBad;

  float mHalfBaseline; // Only for visualization

  Map* mpMap;

  std::mutex mMutexPose;
  std::mutex mMutexConnections;
  std::mutex mMutexFeatures;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_KEYFRAME_H
