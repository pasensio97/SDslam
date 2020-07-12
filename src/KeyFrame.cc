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

#include "KeyFrame.h"
#include "ORBmatcher.h"

using std::vector;
using std::set;
using std::list;
using std::map;
using std::mutex;
using std::unique_lock;

namespace SD_SLAM {

long unsigned int KeyFrame::nNextId = 0;

KeyFrame::KeyFrame(Frame &F, Map *pMap):
  mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
  mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
  mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
  mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
  fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
  mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
  mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
  mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
  mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
  mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
  mnMaxY(F.mnMaxY), mK(F.mK), mTimestamp(F.mTimestamp), mvpMapPoints(F.mvpMapPoints),
  mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
  mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap),
  _is_fake(false) {
  mnId=nNextId++;

  mGrid.resize(mnGridCols);
  for (int i = 0; i<mnGridCols; i++) {
    mGrid[i].resize(mnGridRows);
    for (int j = 0; j < mnGridRows; j++)
      mGrid[i][j] = F.mGrid[i][j];
  }

  SetPose(F.mTcw);
  set_gps_pose(F.gps_pose);
  scale_gps = 0.0;
  
  // Copy pyramid
  int size = F.mvImagePyramid.size();
  mvImagePyramid.resize(size);
  for (int i = 0; i < size; i++)
    mvImagePyramid[i] = F.mvImagePyramid[i].clone();

  mDepthImage = F.mDepthImage.clone();
}

void KeyFrame::SetID(int n) {
  mnId = n;
  nNextId = std::max(nNextId, mnId+1);
}

void KeyFrame::SetPose(const Eigen::Matrix4d &Tcw_) {
  unique_lock<mutex> lock(mMutexPose);

  Eigen::Matrix4d m = Tcw_; // Somehow it fixes problems with Eigen
  Tcw = m;

  Eigen::Matrix3d Rcw = Tcw.block<3, 3>(0, 0);
  Eigen::Matrix3d Rwc = Rcw.transpose();
  Eigen::Vector3d tcw = Tcw.block<3, 1>(0, 3);
  Ow = -Rwc*tcw;

  Twc.setIdentity();
  Twc.block<3, 3>(0, 0) = Rwc;
  Twc.block<3, 1>(0, 3) = Ow;
}

Eigen::Matrix4d KeyFrame::GetPose() {
  unique_lock<mutex> lock(mMutexPose);
  return Tcw;
}

Eigen::Matrix4d KeyFrame::GetPoseInverse() {
  unique_lock<mutex> lock(mMutexPose);
  return Twc;
}

Eigen::Vector3d KeyFrame::GetCameraCenter() {
  unique_lock<mutex> lock(mMutexPose);
  return Ow;
}

Eigen::Matrix3d KeyFrame::GetRotation() {
  unique_lock<mutex> lock(mMutexPose);
  return Tcw.block<3, 3>(0, 0);
}

Eigen::Vector3d KeyFrame::GetTranslation() {
  unique_lock<mutex> lock(mMutexPose);
  return Tcw.block<3, 1>(0, 3);
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight) {
  {
    unique_lock<mutex> lock(mMutexConnections);
    if (!mConnectedKeyFrameWeights.count(pKF))
      mConnectedKeyFrameWeights[pKF]=weight;
    else if (mConnectedKeyFrameWeights[pKF]!=weight)
      mConnectedKeyFrameWeights[pKF]=weight;
    else
      return;
  }

  UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles() {
  unique_lock<mutex> lock(mMutexConnections);
  vector<std::pair<int,KeyFrame*> > vPairs;
  vPairs.reserve(mConnectedKeyFrameWeights.size());
  for (map<KeyFrame*, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
     vPairs.push_back(std::make_pair(mit->second, mit->first));

  std::sort(vPairs.begin(), vPairs.end());
  list<KeyFrame*> lKFs;
  list<int> lWs;
  for (size_t i = 0, iend=vPairs.size(); i < iend; i++) {
    lKFs.push_front(vPairs[i].second);
    lWs.push_front(vPairs[i].first);
  }

  mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
  mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames() {
  unique_lock<mutex> lock(mMutexConnections);
  set<KeyFrame*> s;
  for (map<KeyFrame*, int>::iterator mit = mConnectedKeyFrameWeights.begin();mit != mConnectedKeyFrameWeights.end();mit++)
    s.insert(mit->first);
  return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames() {
  unique_lock<mutex> lock(mMutexConnections);
  return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N) {
  unique_lock<mutex> lock(mMutexConnections);
  if ((int)mvpOrderedConnectedKeyFrames.size()<N)
    return mvpOrderedConnectedKeyFrames;
  else
    return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+N);
}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w) {
  unique_lock<mutex> lock(mMutexConnections);

  if (mvpOrderedConnectedKeyFrames.empty())
    return vector<KeyFrame*>();

  vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w,KeyFrame::weightComp);
  if (it == mvOrderedWeights.end())
    return vector<KeyFrame*>();
  else {
    int n = it-mvOrderedWeights.begin();
    return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
  }
}

int KeyFrame::GetWeight(KeyFrame *pKF) {
  unique_lock<mutex> lock(mMutexConnections);
  if (mConnectedKeyFrameWeights.count(pKF))
    return mConnectedKeyFrameWeights[pKF];
  else
    return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx) {
  unique_lock<mutex> lock(mMutexFeatures);
  mvpMapPoints[idx]=pMP;
}

int KeyFrame::AddMapPoint(MapPoint* pMP, const Eigen::Vector2d &pos) {
  bool found = false;
  int index = -1;
  int size = mvKeys.size();


  // Get index of feature detected at selected position
  for (int i = 0; i < size && !found; i++) {
    const cv::KeyPoint &kp = mvKeys[i];
    if (kp.pt.x == pos(0) && kp.pt.y == pos(1)) {
      index = i;
      found = true;
    }
  }

  if (index >= 0)
    AddMapPoint(pMP, index);

  return index;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx) {
  unique_lock<mutex> lock(mMutexFeatures);
  mvpMapPoints[idx] = static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP) {
  int idx = pMP->GetIndexInKeyFrame(this);
  if (idx >= 0)
    mvpMapPoints[idx] = static_cast<MapPoint*>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP) {
  mvpMapPoints[idx]=pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints() {
  unique_lock<mutex> lock(mMutexFeatures);
  set<MapPoint*> s;
  for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
    if (!mvpMapPoints[i])
      continue;
    MapPoint* pMP = mvpMapPoints[i];
    if (!pMP->isBad())
      s.insert(pMP);
  }
  return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs) {
  unique_lock<mutex> lock(mMutexFeatures);

  int nPoints = 0;
  const bool bCheckObs = minObs > 0;
  for (int i = 0; i < N; i++) {
    MapPoint* pMP = mvpMapPoints[i];
    if (pMP) {
      if (!pMP->isBad()) {
        if (bCheckObs) {
          if (mvpMapPoints[i]->Observations() >= minObs)
            nPoints++;
        } else
          nPoints++;
      }
    }
  }

  return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches() {
  unique_lock<mutex> lock(mMutexFeatures);
  return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx) {
  unique_lock<mutex> lock(mMutexFeatures);
  return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections(bool checkID) {
  map<KeyFrame*, int> KFcounter;

  vector<MapPoint*> vpMP;

  {
    unique_lock<mutex> lockMPs(mMutexFeatures);
    vpMP = mvpMapPoints;
  }

  //For all map points in keyframe check in which other keyframes are they seen
  //Increase counter for those keyframes
  for (vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++) {
    MapPoint* pMP = *vit;

    if (!pMP)
      continue;

    if (pMP->isBad())
      continue;

    map<KeyFrame*, size_t> observations = pMP->GetObservations();

    for (map<KeyFrame*, size_t>::iterator mit=observations.begin(), mend=observations.end(); mit != mend; mit++) {
      if (mit->first->mnId == mnId)
        continue;

      // Use only KFs previous to current KF
      if (checkID && mit->first->mnId > mnId)
        continue;

      KFcounter[mit->first]++;
    }
  }

  // This should not happen
  if (KFcounter.empty())
    return;

  //If the counter is greater than threshold add connection
  //In case no keyframe counter is over threshold add the one with maximum counter
  int nmax = 0;
  KeyFrame* pKFmax=NULL;
  int th = 15;

  vector<std::pair<int,KeyFrame*> > vPairs;
  vPairs.reserve(KFcounter.size());
  for (map<KeyFrame*, int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit != mend; mit++) {
    if (mit->second>nmax) {
      nmax = mit->second;
      pKFmax = mit->first;
    }
    if (mit->second>=th) {
      vPairs.push_back(std::make_pair(mit->second, mit->first));
      (mit->first)->AddConnection(this, mit->second);
    }
  }

  if (vPairs.empty()) {
    vPairs.push_back(std::make_pair(nmax,pKFmax));
    pKFmax->AddConnection(this,nmax);
  }

  std::sort(vPairs.begin(), vPairs.end());
  list<KeyFrame*> lKFs;
  list<int> lWs;
  for (size_t i = 0; i < vPairs.size(); i++) {
    lKFs.push_front(vPairs[i].second);
    lWs.push_front(vPairs[i].first);
  }

  {
    unique_lock<mutex> lockCon(mMutexConnections);
    mConnectedKeyFrameWeights = KFcounter;
    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

    if (mbFirstConnection && mnId != 0) {
      mpParent = mvpOrderedConnectedKeyFrames.front();
      mpParent->AddChild(this);
      mbFirstConnection = false;
    }
  }
}

void KeyFrame::AddChild(KeyFrame *pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  // Avoid linking to itself
  if (pKF->GetID() != GetID()) {
    mpParent = pKF;
    pKF->AddChild(this);
  }
}

set<KeyFrame*> KeyFrame::GetChilds() {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mspChildrens;
}

KeyFrame* KeyFrame::GetParent() {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mbNotErase = true;
  mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges() {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mspLoopEdges;
}

void KeyFrame::SetNotErase() {
  unique_lock<mutex> lock(mMutexConnections);
  mbNotErase = true;
}

void KeyFrame::SetErase() {
  {
    unique_lock<mutex> lock(mMutexConnections);
    if (mspLoopEdges.empty()) {
      mbNotErase = false;
    }
  }

  if (mbToBeErased) {
    SetBadFlag();
  }
}

void KeyFrame::SetBadFlag() {
  {
    unique_lock<mutex> lock(mMutexConnections);
    if (mnId == 0)
      return;
    else if (mbNotErase) {
      mbToBeErased = true;
      return;
    }
  }

  for (map<KeyFrame*, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
    mit->first->EraseConnection(this);

  for (size_t i = 0; i<mvpMapPoints.size(); i++)
    if (mvpMapPoints[i])
      mvpMapPoints[i]->EraseObservation(this);
  {
    unique_lock<mutex> lock(mMutexConnections);
    unique_lock<mutex> lock1(mMutexFeatures);

    mConnectedKeyFrameWeights.clear();
    mvpOrderedConnectedKeyFrames.clear();

    // Update Spanning Tree
    set<KeyFrame*> sParentCandidates;
    sParentCandidates.insert(mpParent);

    // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
    // Include that children as new parent candidate for the rest
    while (!mspChildrens.empty()) {
      bool bContinue = false;

      int max = -1;
      KeyFrame* pC;
      KeyFrame* pP;

      for (set<KeyFrame*>::iterator sit = mspChildrens.begin(), send = mspChildrens.end(); sit != send; sit++) {
        KeyFrame* pKF = *sit;
        if (pKF->isBad())
          continue;

        // Check if a parent candidate is connected to the keyframe
        vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
        for (size_t i = 0, iend=vpConnected.size(); i < iend; i++) {
          for (set<KeyFrame*>::iterator spcit = sParentCandidates.begin(), spcend = sParentCandidates.end(); spcit != spcend; spcit++) {
            if (vpConnected[i]->mnId == (*spcit)->mnId) {
              int w = pKF->GetWeight(vpConnected[i]);
              if (w>max) {
                pC = pKF;
                pP = vpConnected[i];
                max = w;
                bContinue = true;
              }
            }
          }
        }
      }

      if (bContinue) {
        pC->ChangeParent(pP);
        sParentCandidates.insert(pC);
        mspChildrens.erase(pC);
      } else
        break;
    }

    // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
    if (!mspChildrens.empty())
      for (set<KeyFrame*>::iterator sit = mspChildrens.begin(); sit != mspChildrens.end(); sit++) {
        (*sit)->ChangeParent(mpParent);
      }

    mpParent->EraseChild(this);
    mbBad = true;
  }

  mpMap->EraseKeyFrame(this);
}

bool KeyFrame::isBad() {
  unique_lock<mutex> lock(mMutexConnections);
  return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF) {
  bool bUpdate = false;
  {
    unique_lock<mutex> lock(mMutexConnections);
    if (mConnectedKeyFrameWeights.count(pKF)) {
      mConnectedKeyFrameWeights.erase(pKF);
      bUpdate=true;
    }
  }

  if (bUpdate)
    UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
  vector<size_t> vIndices;
  vIndices.reserve(N);

  const int nMinCellX = std::max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
  if (nMinCellX >= mnGridCols)
    return vIndices;

  const int nMaxCellX = std::min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
  if (nMaxCellX < 0)
    return vIndices;

  const int nMinCellY = std::max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
  if (nMinCellY >= mnGridRows)
    return vIndices;

  const int nMaxCellY = std::min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
  if (nMaxCellY < 0)
    return vIndices;

  for (int ix = nMinCellX; ix<=nMaxCellX; ix++) {
    for (int iy = nMinCellY; iy<=nMaxCellY; iy++) {
      const vector<size_t> vCell = mGrid[ix][iy];
      for (size_t j = 0, jend=vCell.size(); j < jend; j++) {
        const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
        const float distx = kpUn.pt.x-x;
        const float disty = kpUn.pt.y-y;

        if (fabs(distx)<r && fabs(disty)<r)
          vIndices.push_back(vCell[j]);
      }
    }
  }

  return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
  return (x >= mnMinX && x<mnMaxX && y >= mnMinY && y<mnMaxY);
}

Eigen::Vector3d KeyFrame::UnprojectStereo(int i) {
  const float z = mvDepth[i];
  if (z > 0) {
    const float u = mvKeys[i].pt.x;
    const float v = mvKeys[i].pt.y;
    const float x = (u-cx)*z*invfx;
    const float y = (v-cy)*z*invfy;
    Eigen::Vector3d x3Dc(x, y, z);

    unique_lock<mutex> lock(mMutexPose);
    return Twc.block<3, 3>(0, 0)*x3Dc+Twc.block<3, 1>(0, 3);
  } else
    return Eigen::Vector3d::Zero();
}

float KeyFrame::ComputeSceneMedianDepth(const int q) {
  vector<MapPoint*> vpMapPoints;
  Eigen::Matrix4d Tcw_;
  {
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPose);
    vpMapPoints = mvpMapPoints;
    Tcw_ = Tcw;
  }

  vector<float> vDepths;
  vDepths.reserve(N);
  Eigen::Vector3d Rcw2(Tcw_(2, 0), Tcw_(2, 1), Tcw_(2, 2));
  float zcw = Tcw_(2, 3);

  for (int i = 0; i < N; i++) {
    if (mvpMapPoints[i]) {
      MapPoint* pMP = mvpMapPoints[i];
      Eigen::Vector3d x3Dw = pMP->GetWorldPos();
      float z = Rcw2.dot(x3Dw)+zcw;
      vDepths.push_back(z);
    }
  }

  std::sort(vDepths.begin(), vDepths.end());

  return vDepths[(vDepths.size()-1)/q];
}

}  // namespace SD_SLAM
