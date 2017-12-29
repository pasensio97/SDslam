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

#include "FrameDrawer.h"
#include "Config.h"

using std::vector;
using std::mutex;

namespace SD_SLAM {

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap) {
  mState=Tracking::SYSTEM_NOT_READY;
  mIm = cv::Mat(Config::Height(), Config::Width(), CV_8UC3, cv::Scalar(0, 0, 0));
}

cv::Mat FrameDrawer::DrawFrame() {
  cv::Mat im;
  vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
  vector<int> vMatches; // Initialization: correspondeces with reference keypoints
  vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
  vector<bool> vbMap; // Tracked MapPoints in current frame
  int state; // Tracking state
  std::vector<cv::Point> ARPoints; // Initial plane

  //Copy variables within scoped mutex
  {
    std::unique_lock<mutex> lock(mMutex);
    state = mState;
    if (mState==Tracking::SYSTEM_NOT_READY)
      mState=Tracking::NO_IMAGES_YET;

    mIm.copyTo(im);

    if (mState==Tracking::NOT_INITIALIZED) {
      vCurrentKeys = mvCurrentKeys;
      vIniKeys = mvIniKeys;
      vMatches = mvIniMatches;
    } else if (mState==Tracking::OK) {
      vCurrentKeys = mvCurrentKeys;
      vbMap = mvbMap;
      ARPoints = ARPoints_;
    } else if (mState==Tracking::LOST) {
      vCurrentKeys = mvCurrentKeys;
    }
  }

  if (im.channels()<3) //this should be always true
    cvtColor(im, im, CV_GRAY2BGR);

  //Draw
  if (state==Tracking::NOT_INITIALIZED) { //INITIALIZING
    for (unsigned int i = 0; i < vMatches.size(); i++) {
      if (vMatches[i] >= 0)
        cv::line(im, vIniKeys[i].pt, vCurrentKeys[vMatches[i]].pt, cv::Scalar(0, 255, 0), 2);
    }    
  } else if (state==Tracking::OK) { //TRACKING
    // Show plane grid
    for (auto it=ARPoints.begin(); it!=ARPoints.end(); it+=2)
      cv::line(im, *it, *(it+1), cv::Scalar(150, 150, 150), 2);

    mnTracked = 0;
    const float r = 3;
    const int n = vCurrentKeys.size();
    for (int i = 0; i<n; i++) {
      if (vbMap[i]) {
        // This is a match to a MapPoint in the map
        cv::circle(im, vCurrentKeys[i].pt, r, cv::Scalar(0, 255, 0), 2);
        mnTracked++;
      }
    }
  }

  cv::Mat imWithInfo;
  DrawTextInfo(im, state, imWithInfo);

  return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText) {
  std::stringstream s;
  if (nState==Tracking::NO_IMAGES_YET)
    s << " WAITING FOR IMAGES";
  else if (nState==Tracking::NOT_INITIALIZED)
    s << " TRYING TO INITIALIZE ";
  else if (nState==Tracking::OK) {
    int nKFs = mpMap->KeyFramesInMap();
    int nMPs = mpMap->MapPointsInMap();
    s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
  } else if (nState==Tracking::LOST) {
    s << " TRACK LOST. TRYING TO RELOCALIZE ";
  } else if (nState==Tracking::SYSTEM_NOT_READY) {
    s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
  }

  int baseline = 0;
  cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1,&baseline);

  im.copyTo(imText);
  imText.rowRange(imText.rows-(textSize.height+10), imText.rows) = cv::Mat::zeros(textSize.height+10, im.cols, im.type());
  cv::putText(imText, s.str(), cv::Point(5, imText.rows-5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
}

void FrameDrawer::Update(Tracking *pTracker) {
  bool showGrid = false;

  std::unique_lock<mutex> lock(mMutex);
  pTracker->GetImage().copyTo(mIm);
  Frame &currentFrame = pTracker->GetCurrentFrame();
  mvCurrentKeys = currentFrame.mvKeys;
  N = mvCurrentKeys.size();
  mvbMap = vector<bool>(N, false);

  if (pTracker->GetLastState() == Tracking::NOT_INITIALIZED)  {
    mvIniKeys=pTracker->GetInitialFrame().mvKeys;
    mvIniMatches=pTracker->GetInitialMatches();
  } else if (pTracker->GetLastState() == Tracking::OK) {
    for (int i = 0; i < N; i++) {
      MapPoint* pMP = currentFrame.mvpMapPoints[i];
      if (pMP) {
        if (!currentFrame.mvbOutlier[i])
          mvbMap[i]=true;
      }
    }
  }
  mState = static_cast<int>(pTracker->GetLastState());

  // Save initial plane positions
  if (showGrid)
    GetInitialPlane(pTracker);
}

void FrameDrawer::GetInitialPlane(Tracking *pTracker) {
  double rmin = -0.5;
  double rmax = 0.5;
  double step = 0.1;
  Eigen::Vector3d p1, p2;
  Eigen::Vector2d p1i, p2i;

  Frame &currentFrame = pTracker->GetCurrentFrame();
  Eigen::Matrix<double, 3, 4> planeRT = pTracker->GetPlaneRT();

  ARPoints_.clear();
  for (double i=rmin; i <= rmax; i += step) {
    for (double j = rmin; j <= rmax; j += step) {
      p1 << i, j, 0.0;
      p2 << i+step, j, 0.0;
      if (Project(currentFrame, planeRT, p1, p1i) && Project(currentFrame, planeRT, p2, p2i)) {
        ARPoints_.push_back(cv::Point(p1i(0), p1i(1)));
        ARPoints_.push_back(cv::Point(p2i(0), p2i(1)));
      }

      p1 << i+step, j, 0.0;
      p2 << i+step, j+step, 0.0;
      if (Project(currentFrame, planeRT, p1, p1i) && Project(currentFrame, planeRT, p2, p2i)) {
        ARPoints_.push_back(cv::Point(p1i(0), p1i(1)));
        ARPoints_.push_back(cv::Point(p2i(0), p2i(1)));
      }

      p1 << i+step, j+step, 0.0;
      p2 << i, j+step, 0.0;
      if (Project(currentFrame, planeRT, p1, p1i) && Project(currentFrame, planeRT, p2, p2i)) {
        ARPoints_.push_back(cv::Point(p1i(0), p1i(1)));
        ARPoints_.push_back(cv::Point(p2i(0), p2i(1)));
      }

      p1 << i, j+step, 0.0;
      p2 << i, j, 0.0;
      if (Project(currentFrame, planeRT, p1, p1i) && Project(currentFrame, planeRT, p2, p2i)) {
        ARPoints_.push_back(cv::Point(p1i(0), p1i(1)));
        ARPoints_.push_back(cv::Point(p2i(0), p2i(1)));
      }
    }
  }
}

bool FrameDrawer::Project(const Frame &frame, const Eigen::Matrix<double, 3, 4> &planeRT, const Eigen::Vector3d &p3d, Eigen::Vector2d &p2d) {
  Eigen::Matrix3d Rcw = frame.GetPose().block<3, 3>(0, 0);
  Eigen::Vector3d tcw = frame.GetPose().block<3, 1>(0, 3);

  Eigen::Vector3d p, pc;

  p = planeRT.block<3, 3>(0, 0)*p3d + planeRT.block<3, 1>(0, 3);
  pc = Rcw*p+tcw;

  const float invzc = 1.0/pc(2);

  if (invzc >= 0) {
    p2d(0) = frame.fx*pc(0)*invzc + frame.cx;
    p2d(1) = frame.fy*pc(1)*invzc + frame.cy;

    if (p2d(0) < -2000 || p2d(0) > 2000 || p2d(1) < -2000 || p2d(1) > 2000)
      return false;

    return true;
  }

  return false;
}

}  // namespace SD_SLAM
