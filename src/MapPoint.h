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

#ifndef SD_SLAM_MAPPOINT_H
#define SD_SLAM_MAPPOINT_H

#include <mutex>
#include <opencv2/core/core.hpp>
#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"

namespace SD_SLAM {

class KeyFrame;
class Map;
class Frame;

class MapPoint {
 public:
  MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
  MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

  void SetWorldPos(const cv::Mat &Pos);
  cv::Mat GetWorldPos();

  cv::Mat GetNormal();
  KeyFrame* GetReferenceKeyFrame();

  std::map<KeyFrame*,size_t> GetObservations();
  int Observations();

  void AddObservation(KeyFrame* pKF,size_t idx);
  void EraseObservation(KeyFrame* pKF);

  int GetIndexInKeyFrame(KeyFrame* pKF);
  bool IsInKeyFrame(KeyFrame* pKF);

  void SetBadFlag();
  bool isBad();

  void Replace(MapPoint* pMP);
  MapPoint* GetReplaced();

  void IncreaseVisible(int n=1);
  void IncreaseFound(int n=1);
  float GetFoundRatio();
  inline int GetFound(){
    return mnFound;
  }

  void ComputeDistinctiveDescriptors();

  cv::Mat GetDescriptor();

  void UpdateNormalAndDepth();

  float GetMinDistanceInvariance();
  float GetMaxDistanceInvariance();
  int PredictScale(const float &currentDist, KeyFrame*pKF);
  int PredictScale(const float &currentDist, Frame* pF);

 public:
  long unsigned int mnId;
  static long unsigned int nNextId;
  long int mnFirstKFid;
  long int mnFirstFrame;
  int nObs;

  // Variables used by the tracking
  float mTrackProjX;
  float mTrackProjY;
  float mTrackProjXR;
  bool mbTrackInView;
  int mnTrackScaleLevel;
  float mTrackViewCos;
  long unsigned int mnTrackReferenceForFrame;
  long unsigned int mnLastFrameSeen;

  // Variables used by local mapping
  long unsigned int mnBALocalForKF;
  long unsigned int mnFuseCandidateForKF;

  // Variables used by loop closing
  long unsigned int mnLoopPointForKF;
  long unsigned int mnCorrectedByKF;
  long unsigned int mnCorrectedReference;
  cv::Mat mPosGBA;
  long unsigned int mnBAGlobalForKF;


  static std::mutex mGlobalMutex;

 protected:
   // Position in absolute coordinates
   cv::Mat mWorldPos;

   // Keyframes observing the point and associated index in keyframe
   std::map<KeyFrame*,size_t> mObservations;

   // Mean viewing direction
   cv::Mat mNormalVector;

   // Best descriptor to fast matching
   cv::Mat mDescriptor;

   // Reference KeyFrame
   KeyFrame* mpRefKF;

   // Tracking counters
   int mnVisible;
   int mnFound;

   // Bad flag (we do not currently erase MapPoint from memory)
   bool mbBad;
   MapPoint* mpReplaced;

   // Scale invariance distances
   float mfMinDistance;
   float mfMaxDistance;

   Map* mpMap;

   std::mutex mMutexPos;
   std::mutex mMutexFeatures;
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_MAPPOINT_H
