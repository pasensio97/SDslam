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

System::System(const eSensor sensor, bool loopClosing): mSensor(sensor), mbReset(false) {
  if (mSensor==MONOCULAR) {
    LOGD("Input sensor was set to Monocular");
  } else if (mSensor==RGBD) {
    LOGD("Input sensor was set to RGB-D");
  } else if (mSensor==MONOCULAR_IMU) {
    LOGD("Input sensor was set to Monocular-IMU");
  }

  // Create the Map
  mpMap = new Map();

  // Initialize the Tracking thread (it will live in the main thread of execution)
  mpTracker = new Tracking(this, mpMap, mSensor);

  // Initialize the Local Mapping thread and launch
  mpLocalMapper = new LocalMapping(mpMap, mSensor!=RGBD);
  mptLocalMapping = new std::thread(&SD_SLAM::LocalMapping::Run, mpLocalMapper);

  // Initialize the Loop Closing thread and launch
  if (loopClosing) {
    LOGD("Loop closing activated");
    mpLoopCloser = new LoopClosing(mpMap, mSensor==RGBD);
    mptLoopClosing = new std::thread(&SD_SLAM::LoopClosing::Run, mpLoopCloser);
  } else {
    LOGD("Loop closing not activated");
    mpLoopCloser = nullptr;
    mptLoopClosing = nullptr;
  }

  // Set pointers between threads
  mpTracker->SetLocalMapper(mpLocalMapper);
  mpLocalMapper->SetTracker(mpTracker);

  if (loopClosing) {
    mpTracker->SetLoopClosing(mpLoopCloser);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);
    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
  }
}

Eigen::Matrix4d System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const std::string filename) {
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

  Eigen::Matrix4d Tcw = mpTracker->GrabImageRGBD(im, depthmap, filename);

  total.Stop();
  LOGD("Tracking time is %.2fms", total.GetMsTime());

  LOGD("Pose: [%.4f, %.4f, %.4f]", Tcw(0, 3), Tcw(1, 3), Tcw(2, 3));

  unique_lock<mutex> lock2(mMutexState);
  mTrackingState = mpTracker->GetState();
  mTrackedMapPoints = mpTracker->GetCurrentFrame().mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->GetCurrentFrame().mvKeysUn;
  return Tcw;
}

Eigen::Matrix4d System::TrackMonocular(const cv::Mat &im, const std::string filename) {
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

  Eigen::Matrix4d Tcw = mpTracker->GrabImageMonocular(im, filename);

  total.Stop();
  LOGD("Tracking time is %.2fms", total.GetMsTime());

  LOGD("Pose: [%.4f, %.4f, %.4f]", Tcw(0, 3), Tcw(1, 3), Tcw(2, 3));

  unique_lock<mutex> lock2(mMutexState);
  mTrackingState = mpTracker->GetState();
  mTrackedMapPoints = mpTracker->GetCurrentFrame().mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->GetCurrentFrame().mvKeysUn;

  return Tcw;
}

Eigen::Matrix4d System::TrackFusion(const cv::Mat &im, const vector<double> &measurements, const std::string filename) {
  LOGD("Track monocular image with other sensor measurements");

  if (mSensor!=MONOCULAR_IMU) {
    LOGE("Called TrackFusion but input sensor was not set to Monocular-IMU");
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

  mpTracker->SetMeasurements(measurements);
  Eigen::Matrix4d Tcw = mpTracker->GrabImageMonocular(im, filename);

  total.Stop();
  LOGD("Tracking time is %.2fms", total.GetMsTime());

  LOGD("Pose: [%.4f, %.4f, %.4f]", Tcw(0, 3), Tcw(1, 3), Tcw(2, 3));

  unique_lock<mutex> lock2(mMutexState);
  mTrackingState = mpTracker->GetState();
  mTrackedMapPoints = mpTracker->GetCurrentFrame().mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->GetCurrentFrame().mvKeysUn;

  return Tcw;
}

bool System::MapChanged() {
  static int n = 0;
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
  bool waitLoop = false;

  mpLocalMapper->RequestFinish();
  if (mpLoopCloser) {
    mpLoopCloser->RequestFinish();
    waitLoop = true;
  }

  // Wait until all thread have effectively stopped
  while (!mpLocalMapper->isFinished() || waitLoop) {
    usleep(5000);
    if (mpLoopCloser)
      waitLoop = !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA();
  }

  mptLocalMapping->join();
  if (mptLoopClosing)
    mptLoopClosing->join();
}

void System::SaveTrajectory(const std::string &filename) {
  int counter;
  std::string output = "%YAML:1.0\n";

  std::cout << "Saving trajectory to " << filename << " ..." << std::endl;

  vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  std::ofstream f;
  f.open(filename.c_str());

  // Save keyframes
  output += "keyframes:\n";

  for(size_t i=0; i<vpKFs.size(); i++) {
    KeyFrame* pKF = vpKFs[i];

    if(pKF->isBad())
      continue;

    Eigen::Quaterniond q(pKF->GetRotation());
    Eigen::Vector3d t = pKF->GetTranslation();

    output += "  - id: " + std::to_string(pKF->mnId) + "\n";
    output += "    filename: " + pKF->mFilename + "\n";
    output += "    pose:\n";
    output += "      - " + std::to_string(q.w()) + "\n";
    output += "      - " + std::to_string(q.x()) + "\n";
    output += "      - " + std::to_string(q.y()) + "\n";
    output += "      - " + std::to_string(q.z()) + "\n";
    output += "      - " + std::to_string(t(0)) + "\n";
    output += "      - " + std::to_string(t(1)) + "\n";
    output += "      - " + std::to_string(t(2)) + "\n";
  }

  // Save map points
  output += "points:\n";
  counter = 0;

  const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
  for (size_t i = 0, iend=vpMPs.size(); i < iend; i++) {
    if (vpMPs[i]->isBad())
      continue;
    Eigen::Vector3d pos = vpMPs[i]->GetWorldPos();

    output += "  - id: " + std::to_string(counter) + "\n";
    output += "    pose:\n";
    output += "      - " + std::to_string(pos(0)) + "\n";
    output += "      - " + std::to_string(pos(1)) + "\n";
    output += "      - " + std::to_string(pos(2)) + "\n";
    output += "    observations:\n";

    // Observations
    std::map<KeyFrame*, size_t> observations = vpMPs[i]->GetObservations();

    for (std::map<KeyFrame*, size_t>::iterator mit=observations.begin(), mend=observations.end(); mit != mend; mit++) {
      KeyFrame* kf = mit->first;
      const cv::KeyPoint &kp = kf->mvKeys[mit->second];

      output += "      - kf: " + std::to_string(kf->mnId) + "\n";
      output += "        point:\n";
      output += "          - "+ std::to_string(kp.pt.x) + "\n";
      output += "          - "+ std::to_string(kp.pt.y) + "\n";
    }

    counter++;
  }

  f << output;
  f.close();
  std::cout << "Trajectory saved!" << std::endl;
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
