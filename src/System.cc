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

#include "System.h"
#include <iomanip>
#include <unistd.h>
#include "Config.h"
#include "extra/timer.h"
#include "extra/log.h"

using std::mutex;
using std::unique_lock;
using std::vector;
using std::string;
using std::cout;
using std::endl;

namespace SD_SLAM {

System::System(const eSensor sensor): mSensor(sensor), mbReset(false) {

  cout << "Input sensor was set to: ";
  if (mSensor==MONOCULAR)
    cout << "Monocular" << endl;
  else if (mSensor==RGBD)
    cout << "RGB-D" << endl;

  // Create the Map
  mpMap = new Map();

  // Initialize the Tracking thread (it will live in the main thread of execution)
  mpTracker = new Tracking(this, mpMap, mSensor);

  // Initialize the Local Mapping thread and launch
  mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
  mptLocalMapping = new std::thread(&SD_SLAM::LocalMapping::Run, mpLocalMapper);

  // Initialize the Loop Closing thread and launch
  mpLoopCloser = new LoopClosing(mpMap, mSensor!=MONOCULAR);
  mptLoopClosing = new std::thread(&SD_SLAM::LoopClosing::Run, mpLoopCloser);

  // Set pointers between threads
  mpTracker->SetLocalMapper(mpLocalMapper);
  mpTracker->SetLoopClosing(mpLoopCloser);

  mpLocalMapper->SetTracker(mpTracker);
  mpLocalMapper->SetLoopCloser(mpLoopCloser);

  mpLoopCloser->SetTracker(mpTracker);
  mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

Eigen::Matrix4d System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap) {
  LOGD("Track RGBD image");

  if (mSensor!=RGBD) {
    LOGE("Called TrackRGBD but input sensor was not set to RGBD");
    exit(-1);
  }

  // Check reset
  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      mbReset = false;
    }
  }

  Timer total(true);

  Eigen::Matrix4d Tcw = mpTracker->GrabImageRGBD(im,depthmap);

  total.Stop();
  LOGD("Tracking time is %.2fms", total.GetMsTime());

  LOGD("Pose: [%.4f, %.4f, %.4f]", Tcw(0,3), Tcw(1,3), Tcw(2,3));

  unique_lock<mutex> lock2(mMutexState);
  mTrackingState = mpTracker->GetState();
  mTrackedMapPoints = mpTracker->GetCurrentFrame().mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->GetCurrentFrame().mvKeysUn;
  return Tcw;
}

Eigen::Matrix4d System::TrackMonocular(const cv::Mat &im) {
  LOGD("Track monocular image");

  if (mSensor!=MONOCULAR) {
    LOGE("Called TrackMonocular but input sensor was not set to Monocular");
    exit(-1);
  }

  // Check reset
  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      mbReset = false;
    }
  }

  Timer total(true);

  Eigen::Matrix4d Tcw = mpTracker->GrabImageMonocular(im);

  total.Stop();
  LOGD("Tracking time is %.2fms", total.GetMsTime());

  LOGD("Pose: [%.4f, %.4f, %.4f]", Tcw(0,3), Tcw(1,3), Tcw(2,3));

  unique_lock<mutex> lock2(mMutexState);
  mTrackingState = mpTracker->GetState();
  mTrackedMapPoints = mpTracker->GetCurrentFrame().mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->GetCurrentFrame().mvKeysUn;

  return Tcw;
}

bool System::MapChanged() {
  static int n=0;
  int curn = mpMap->GetLastBigChangeIdx();
  if (n<curn) {
    n=curn;
    return true;
  } else
    return false;
}

void System::Reset() {
  unique_lock<mutex> lock(mMutexReset);
  mbReset = true;
}

void System::Shutdown() {
  mpLocalMapper->RequestFinish();
  mpLoopCloser->RequestFinish();

  // Wait until all thread have effectively stopped
  while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA()) {
    usleep(5000);
  }

  mptLocalMapping->join();
  mptLoopClosing->join();
}

int System::GetTrackingState() {
  unique_lock<mutex> lock(mMutexState);
  return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints() {
  unique_lock<mutex> lock(mMutexState);
  return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
  unique_lock<mutex> lock(mMutexState);
  return mTrackedKeyPointsUn;
}

}  // namespace SD_SLAM
