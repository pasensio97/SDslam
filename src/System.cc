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

#include "System.h"
#include <iomanip>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>
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

System::System(const eSensor sensor, bool loopClosing): mSensor(sensor), mbReset(false),
               mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false),
               stopRequested_(false) {
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

  // Check mode change
  {
    unique_lock<mutex> lock(mMutexMode);
    if(mbActivateLocalizationMode) {
      mpLocalMapper->RequestStop();

      // Wait until Local Mapping has effectively stopped
      while(!mpLocalMapper->isStopped()) {
        usleep(1000);
      }

      mpTracker->InformOnlyTracking(true);
      mbActivateLocalizationMode = false;
    }
    if(mbDeactivateLocalizationMode) {
      mpTracker->InformOnlyTracking(false);
      mpLocalMapper->Release();
      mbDeactivateLocalizationMode = false;
    }
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

  // Check mode change
  {
    unique_lock<mutex> lock(mMutexMode);
    if(mbActivateLocalizationMode) {
      mpLocalMapper->RequestStop();

      // Wait until Local Mapping has effectively stopped
      while(!mpLocalMapper->isStopped()) {
        usleep(1000);
      }

      mpTracker->InformOnlyTracking(true);
      mbActivateLocalizationMode = false;
    }
    if(mbDeactivateLocalizationMode) {
      mpTracker->InformOnlyTracking(false);
      mpLocalMapper->Release();
      mbDeactivateLocalizationMode = false;
    }
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

void System::ActivateLocalizationMode() {
  unique_lock<mutex> lock(mMutexMode);
  mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode() {
  unique_lock<mutex> lock(mMutexMode);
  mbDeactivateLocalizationMode = true;
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

void System::SaveTrajectory(const std::string &filename, const std::string &foldername) {
#ifndef ANDROID
  int counter;
  std::string output = "%YAML:1.0\n";

  std::cout << "Saving trajectory to " << filename << " ..." << std::endl;

  // Create directory to store images
  if(mkdir(foldername.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {
    std::cerr << "Warning: Couldn't create folder " << foldername << std::endl;
  }

  vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  std::ofstream f;
  f.open(filename.c_str());

  // Save camera parameters
  output += "camera:\n";
  output += "  fx: " + std::to_string(Config::fx()) + "\n";
  output += "  fy: " + std::to_string(Config::fy()) + "\n";
  output += "  cx: " + std::to_string(Config::cx()) + "\n";
  output += "  cy: " + std::to_string(Config::cy()) + "\n";
  output += "  k1: " + std::to_string(Config::k1()) + "\n";
  output += "  k2: " + std::to_string(Config::k2()) + "\n";
  output += "  p1: " + std::to_string(Config::p1()) + "\n";
  output += "  p2: " + std::to_string(Config::p2()) + "\n";
  output += "  k3: " + std::to_string(Config::k3()) + "\n";

  // Save keyframes
  output += "keyframes:\n";

  for(size_t i=0; i<vpKFs.size(); i++) {
    KeyFrame* pKF = vpKFs[i];

    if(pKF->isBad())
      continue;

    Eigen::Matrix4d pose = pKF->GetPoseInverse();
    Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    Eigen::Vector3d t = pose.block<3, 1>(0, 3);

    // Save images
    string imgname, depthname;
    imgname = foldername + "/" + std::to_string(pKF->mnId) + ".png";
    cv::imwrite(imgname, pKF->mvImagePyramid[0]);

    if (mSensor==RGBD) {
      float depthFactor = 1.0/mpTracker->GetDepthFactor();
      depthname = foldername + "/" + std::to_string(pKF->mnId) + "_depth.png";
      // Restore initial depth image
      pKF->mDepthImage.convertTo(pKF->mDepthImage, CV_16U, depthFactor);
      cv::imwrite(depthname, pKF->mDepthImage);
    }

    output += "  - id: " + std::to_string(pKF->mnId) + "\n";
    output += "    filename: \"" + imgname + "\"\n";
    if (mSensor==RGBD)
      output += "    depthname: \"" + depthname + "\"\n";
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
      output += "        pixel:\n";
      output += "          - "+ std::to_string(kp.pt.x) + "\n";
      output += "          - "+ std::to_string(kp.pt.y) + "\n";
    }

    counter++;
  }

  f << output;
  f.close();
  std::cout << "Trajectory saved!" << std::endl;
#endif // ANDROID
}

// Load saved trajectory
bool System::LoadTrajectory(const std::string &filename) {
#ifndef ANDROID
  cv::FileStorage fs;
  cv::Mat im, imD;

  LOGD("Loading trajectory from file %s", filename.c_str());
  try {
    // Read config file
    fs.open(filename.c_str(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
      LOGE("Failed to open file: %s", filename.c_str());
      return false;
    }
  } catch(cv::Exception &ex) {
    LOGE("Parse error: %s", ex.what());
    return false;
  }

  // Read keyframes
  cv::FileNode keyframes = fs["keyframes"];

  for(auto it = keyframes.begin(); it != keyframes.end(); ++it) {
    int id;
    vector<double> pose;
    std::string filename, depthname;

    (*it)["id"] >> id;
    (*it)["filename"] >> filename;
    if (mSensor==RGBD)
      (*it)["depthname"] >> depthname;
    (*it)["pose"] >> pose;

    if (pose.size() != 7) {
      LOGE("KeyFrame pose not valid");
      continue;
    }

    // Load image
    im = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    if(im.empty()) {
      LOGE("Couldn't load image %s", filename.c_str());
      continue;
    }

    if (mSensor==RGBD) {
      imD = cv::imread(depthname, cv::IMREAD_UNCHANGED);
      if(imD.empty()) {
        LOGE("Couldn't load depth image %s", depthname.c_str());
        continue;
      }
    }

    // Calculate pose
    Eigen::Matrix4d mpose;
    Eigen::Quaterniond q(pose[0], pose[1], pose[2], pose[3]);
    Eigen::Vector3d t(pose[4], pose[5], pose[6]);
    mpose.setIdentity();
    mpose.block<3, 3>(0, 0) = q.toRotationMatrix();
    mpose.block<3, 1>(0, 3) = t;

    Frame frame;
    if (mSensor==RGBD) {
      frame = mpTracker->CreateFrame(im, imD);
    } else {
      frame = mpTracker->CreateFrame(im);
    }
    frame.SetPose(mpose);
    frame.SetPose(frame.GetPoseInverse()); // Saved pose was in "world to camera coordinates"

    KeyFrame* kf = new KeyFrame(frame, mpMap);
    kf->SetID(id);

    // Insert Keyframe in Map
    mpTracker->SetReferenceKeyFrame(kf);
    mpMap->AddKeyFrame(kf);
  }

  // Read map points
  cv::FileNode points = fs["points"];

  for(auto it = points.begin(); it != points.end(); ++it) {
    int id;
    vector<double> position;

    (*it)["id"] >> id;
    (*it)["pose"] >> position;

    if (position.size() != 3) {
      LOGE("Map point pose not valid");
      continue;
    }

    MapPoint * mp = nullptr;

    cv::FileNode observations = (*it)["observations"];

    // Read observations
    for(auto it_obs = observations.begin(); it_obs != observations.end(); ++it_obs) {
      int kf_id;
      vector<double> pixel;

      (*it_obs)["kf"] >> kf_id;
      (*it_obs)["pixel"] >> pixel;

      if (pixel.size() != 2) {
        LOGE("Map point pixel not valid");
        continue;
      }

      // Search keyFrame by id
      KeyFrame * kf = mpMap->GetKeyFrame(kf_id);
      if (kf == nullptr)
        continue;

      // Create map point for the first time
      if (mp == nullptr) {
        Eigen::Vector3d worldPos(position[0], position[1], position[2]);
        mp = new MapPoint(worldPos, kf, mpMap);
      }

      Eigen::Vector2d imgPos(pixel[0], pixel[1]);
      int index = kf->AddMapPoint(mp, imgPos);
      if (index >= 0) {
        mp->AddObservation(kf, index);
        mp->ComputeDistinctiveDescriptors();
        mp->UpdateNormalAndDepth();

        //Add to Map
        mpMap->AddMapPoint(mp);
      }

    }
  }

  // Update links in the Covisibility Graph
  mpMap->UpdateConnections();

  LOGD("Map loaded!");

  fs.release();

  // Force relocalization inside loaded map
  mpTracker->ForceRelocalization();
  //ActivateLocalizationMode();

#endif // ANDROID

  return true;
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
