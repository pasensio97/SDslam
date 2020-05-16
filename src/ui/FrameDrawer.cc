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

#include "FrameDrawer.h"
#include "Config.h"
#include "extra/utils.h"

using std::vector;
using std::mutex;
using std::unique_lock;

namespace SD_SLAM {

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap) {
  mState = Tracking::SYSTEM_NOT_READY;
  mIm = cv::Mat(Config::Height(), Config::Width(), CV_8UC3, cv::Scalar(0, 0, 0));
  undistort = false;
  addPlane = false;
  clearAR = false;
  onlyTracking_ = false;
}

cv::Mat FrameDrawer::DrawFrame() {
  cv::Mat im;
  vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
  vector<int> vMatches; // Initialization: correspondeces with reference keypoints
  vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
  vector<bool> vbMap; // Tracked MapPoints in current frame
  int state; // Tracking state

  //Copy variables within scoped mutex
  {
    unique_lock<mutex> lock(mMutex);
    state = mState;
    if (mState == Tracking::SYSTEM_NOT_READY)
      mState = Tracking::NO_IMAGES_YET;

    mIm.copyTo(im);

    if (mState == Tracking::NOT_INITIALIZED) {
      vCurrentKeys = mvCurrentKeys;
      vIniKeys = mvIniKeys;
      vMatches = mvIniMatches;
    } else if (mState == Tracking::OK) {
      vCurrentKeys = mvCurrentKeys;
      vbMap = mvbMap;
    } else if (mState == Tracking::LOST) {
      vCurrentKeys = mvCurrentKeys;
    }
  }

  if (im.channels()<3)
    cvtColor(im, im, CV_GRAY2BGR);

  //Draw
  if (state==Tracking::NOT_INITIALIZED) { //INITIALIZING
    for (unsigned int i = 0; i < vMatches.size(); i++) {
      if (vMatches[i] >= 0)
        cv::line(im, vIniKeys[i].pt, vCurrentKeys[vMatches[i]].pt, cv::Scalar(0, 255, 0), 2);
    }
  } else if (state==Tracking::OK) { //TRACKING
    mnTracked = 0;
    const float r = 3;
    const int n = vCurrentKeys.size();
    for (int i = 0; i<n; i++) {
      if (vbMap[i]) {
        // This is a match to a MapPoint in the map
        if (onlyTracking_)
          cv::circle(im, vCurrentKeys[i].pt, r, cv::Scalar(0, 0, 255), 2);
        else
          cv::circle(im, vCurrentKeys[i].pt, r, cv::Scalar(0, 255, 0), 2);
        mnTracked++;
      }
    }
  }

  DrawTextInfo(im, state);

  return im;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState) {
  std::stringstream s;
  if (nState==Tracking::NO_IMAGES_YET)
    s << " WAITING FOR IMAGES";
  else if (nState==Tracking::NOT_INITIALIZED)
    s << " TRYING TO INITIALIZE ";
  else if (nState==Tracking::OK) {
    if(onlyTracking_)
      s << "LOCALIZATION | ";
    int nKFs = mpMap->KeyFramesInMap();
    int nMPs = mpMap->MapPointsInMap();
    s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
  } else if (nState==Tracking::OK_DIFODO) {
    s << " TRACKING WITH DIFODO (only depth) ";
  } else if (nState==Tracking::LOST) {
    s << " TRACK LOST. TRYING TO RELOCALIZE ";
  } else if (nState==Tracking::SYSTEM_NOT_READY) {
    s << " STARTING...";
  }

  int baseline = 0;
  cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1,&baseline);

  im.rowRange(im.rows-(textSize.height+10), im.rows) = cv::Mat::zeros(textSize.height+10, im.cols, im.type());
  cv::putText(im, s.str(), cv::Point(5, im.rows-5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
}

void FrameDrawer::Update(const cv::Mat &im, const Eigen::Matrix4d &pose, Tracking *pTracker) {
  std::unique_lock<mutex> lock(mMutex);
  mPose = pose;

  Frame &currentFrame = pTracker->GetCurrentFrame();
  if (undistort)
    currentFrame.Undistort(im, mIm);
  else
    im.copyTo(mIm);
  if (undistort)
    mvCurrentKeys = currentFrame.mvKeysUn;
  else
    mvCurrentKeys = currentFrame.mvKeys;
  int n = mvCurrentKeys.size();
  mvbMap = vector<bool>(n, false);
  mvMPs.clear();
  onlyTracking_ = pTracker->OnlyTracking();

  if (pTracker->GetLastState() == Tracking::NOT_INITIALIZED)  {
    if (undistort)
      mvIniKeys = pTracker->GetInitialFrame().mvKeysUn;
    else
      mvIniKeys = pTracker->GetInitialFrame().mvKeys;
    mvIniMatches = pTracker->GetInitialMatches();
  } else if (pTracker->GetLastState() == Tracking::OK) {
    for (int i = 0; i < n; i++) {
      MapPoint* pMP = currentFrame.mvpMapPoints[i];
      if (pMP) {
        // Save points seen
        if (!currentFrame.mvbOutlier[i])
          mvbMap[i] = true;

        // Save best observed points
        if(pMP->Observations() > 5)
          mvMPs.push_back(pMP);
      }
    }
  }
  mState = static_cast<int>(pTracker->GetLastState());
}

void FrameDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M) {
  if (!mPose.isZero()) {
    Eigen::Matrix3d Rwc;
    Eigen::Vector3d twc;
    {
      unique_lock<mutex> lock(mMutex);
      Rwc = mPose.block<3, 3>(0, 0).transpose();
      twc = -Rwc*mPose.block<3, 1>(0, 3);
    }

    M.m[0] = mPose(0, 0);
    M.m[1] = mPose(1, 0);
    M.m[2] = mPose(2, 0);
    M.m[3]  = 0.0;

    M.m[4] = mPose(0, 1);
    M.m[5] = mPose(1, 1);
    M.m[6] = mPose(2, 1);
    M.m[7]  = 0.0;

    M.m[8] = mPose(0, 2);
    M.m[9] = mPose(1, 2);
    M.m[10] = mPose(2, 2);
    M.m[11]  = 0.0;

    M.m[12] = mPose(0, 3);
    M.m[13] = mPose(1, 3);
    M.m[14] = mPose(2, 3);
    M.m[15]  = 1.0;
  } else
    M.SetIdentity();
}

void FrameDrawer::CheckPlanes(bool recompute) {
  int state;
  std::vector<MapPoint*> vMPs;
  Eigen::Matrix4d pose;

  //Copy variables within scoped mutex
  {
    unique_lock<mutex> lock(mMutex);
    state = mState;
    vMPs = mvMPs;
    pose = mPose;
  }


  if(state == Tracking::OK) {
    if(clearAR) {
      if(!vpPlane.empty()) {
        for(size_t i=0; i<vpPlane.size(); i++) {
          delete vpPlane[i];
        }
        vpPlane.clear();
        std::cout << "[DEBUG] Remove AR objects!" << std::endl;
      }
      clearAR = false;
    }

    // Add new plane
    if(addPlane) {
      Plane* pPlane = DetectPlane(pose, vMPs ,50);
      if(pPlane) {
        std::cout << "[DEBUG] New plane detected!" << std::endl;
        vpPlane.push_back(pPlane);
      } else {
        std::cout << "[DEBUG] No plane detected." << std::endl;
      }
      addPlane = false;
    }

    if(!vpPlane.empty()) {
      for(size_t i=0; i<vpPlane.size(); i++) {
        Plane* pPlane = vpPlane[i];

        if(pPlane) {
          if(recompute) {
            pPlane->Recompute();
          }
          glPushMatrix();
          pPlane->glTpw.Multiply();

          // Draw cube
          DrawCube(0.1);

          // Draw grid plane
          DrawPlane(5, 0.1);

          glPopMatrix();
        }
      }
    }


  }
}

Plane* FrameDrawer::DetectPlane(const Eigen::Matrix4d &pose, const std::vector<MapPoint*> &vMPs, const int iterations) {
  // Retrieve 3D points
  vector<Eigen::Vector3d> vPoints;
  vPoints.reserve(vMPs.size());

  for(size_t i=0; i<vMPs.size(); i++) {
    MapPoint* pMP = mvMPs[i];
    vPoints.push_back(pMP->GetWorldPos());
  }

  const int N = vPoints.size();

  if(N<50)
    return NULL;


  // Indices for minimum set selection
  vector<size_t> vAllIndices;
  vAllIndices.reserve(N);
  vector<size_t> vAvailableIndices;

  for(int i=0; i<N; i++)
    vAllIndices.push_back(i);

  float bestDist = 1e10;
  vector<float> bestvDist;

  // RANSAC
  for(int n=0; n<iterations; n++) {
    vAvailableIndices = vAllIndices;

    cv::Mat A(3,4,CV_32F);
    A.col(3) = cv::Mat::ones(3,1,CV_32F);

    // Get min set of points
    for(short i = 0; i < 3; ++i) {
      int randi = Random(0, vAvailableIndices.size()-1);
      int idx = vAvailableIndices[randi];

      A.at<float>(i, 0) = vPoints[idx](0);
      A.at<float>(i, 1) = vPoints[idx](1);
      A.at<float>(i, 2) = vPoints[idx](2);

      vAvailableIndices[randi] = vAvailableIndices.back();
      vAvailableIndices.pop_back();
    }

    cv::Mat u,w,vt;
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    const float a = vt.at<float>(3,0);
    const float b = vt.at<float>(3,1);
    const float c = vt.at<float>(3,2);
    const float d = vt.at<float>(3,3);

    vector<float> vDistances(N,0);

    const float f = 1.0f/sqrt(a*a+b*b+c*c+d*d);

    for(int i=0; i<N; i++)
      vDistances[i] = fabs(vPoints[i](0)*a+vPoints[i](1)*b+vPoints[i](2)*c+d)*f;

    vector<float> vSorted = vDistances;
    sort(vSorted.begin(),vSorted.end());

    int nth = std::max((int)(0.2*N),20);
    const float medianDist = vSorted[nth];

    if(medianDist < bestDist) {
      bestDist = medianDist;
      bestvDist = vDistances;
    }
  }

  // Compute threshold inlier/outlier
  const float th = 1.4*bestDist;
  vector<bool> vbInliers(N,false);
  int nInliers = 0;
  for(int i=0; i<N; i++) {
    if(bestvDist[i]<th) {
      nInliers++;
      vbInliers[i]=true;
    }
  }

  vector<MapPoint*> vInlierMPs(nInliers,NULL);
  int nin = 0;
  for(int i=0; i<N; i++) {
    if(vbInliers[i]) {
      vInlierMPs[nin] = vMPs[i];
      nin++;
    }
  }

  return new Plane(vInlierMPs, pose);
}

void FrameDrawer::DrawCube(const float &size,const float x, const float y, const float z) {
    pangolin::OpenGlMatrix M = pangolin::OpenGlMatrix::Translate(-x,-size-y,-z);
    glPushMatrix();
    M.Multiply();
    pangolin::glDrawColouredCube(-size,size);
    glPopMatrix();
}

void FrameDrawer::DrawPlane(Plane *pPlane, int ndivs, float ndivsize) {
    glPushMatrix();
    pPlane->glTpw.Multiply();
    DrawPlane(ndivs,ndivsize);
    glPopMatrix();
}

void FrameDrawer::DrawPlane(int ndivs, float ndivsize) {
    // Plane parallel to x-z at origin with normal -y
    const float minx = -ndivs*ndivsize;
    const float minz = -ndivs*ndivsize;
    const float maxx = ndivs*ndivsize;
    const float maxz = ndivs*ndivsize;


    glLineWidth(2);
    glColor3f(0.7f,0.7f,1.0f);
    glBegin(GL_LINES);

    for(int n = 0; n<=2*ndivs; n++) {
        glVertex3f(minx+ndivsize*n,0,minz);
        glVertex3f(minx+ndivsize*n,0,maxz);
        glVertex3f(minx,0,minz+ndivsize*n);
        glVertex3f(maxx,0,minz+ndivsize*n);
    }

    glEnd();
}

}  // namespace SD_SLAM
