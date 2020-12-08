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

#ifndef SD_SLAM_SYSTEM_H
#define SD_SLAM_SYSTEM_H

#include <thread>
#include <vector>
#include <opencv2/core/core.hpp>
#include "Tracking.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "inertial/IMU_Measurements.h"

namespace SD_SLAM {

class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System {
 public:
  // Input sensor
  enum eSensor{
    MONOCULAR = 0,
    RGBD = 1,
    MONOCULAR_IMU = 2,
    MONOCULAR_IMU_NEW = 3
  };

 public:
  // Initialize the SLAM system. It launches the Local Mapping and Loop Closing.
  System(const eSensor sensor, bool loopClosing = true);

  inline Map * GetMap() { return mpMap; }
  inline Tracking * GetTracker() { return mpTracker; }

  inline void RequestStop() { stopRequested_ = true; }
  inline bool StopRequested() const { return stopRequested_; }

  // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
  // Input image: Grayscale (CV_8U).
  // Input depthmap: Float (CV_32F).
  // Returns the camera pose (empty if tracking fails).
  Eigen::Matrix4d TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const std::string filename = "");

  // Proccess the given monocular frame
  // Input images: Grayscale (CV_8U).
  // Returns the camera pose (empty if tracking fails).
  Eigen::Matrix4d TrackMonocular(const cv::Mat &im, const std::string filename = "");

  // Proccess the given monocular frame and sensor measurements
  // Input images: Grayscale (CV_8U).
  // Input measurements: Float (CV_32F).
  // Returns the camera pose (empty if tracking fails).
  Eigen::Matrix4d TrackFusion(const cv::Mat &im, const std::vector<double> &measurements, const std::string filename = "");

  // Proccess the given monocular frame and imu measurements
  // Input images: Grayscale (CV_8U).
  // Input measurements: IMU_Measurements.
  // Returns the camera pose (empty if tracking fails).
  Eigen::Matrix4d TrackNewFusion(const cv::Mat &im, const IMU_Measurements &measurements, const double dt, double timestamp=0.0);

  // This stops local mapping thread (map building) and performs only camera tracking.
  void ActivateLocalizationMode();
  // This resumes local mapping thread and performs SLAM again.
  void DeactivateLocalizationMode();

  // Returns true if there have been a big map change (loop closure, global BA)
  // since last call to this function
  bool MapChanged();

  // Reset the system (clear map)
  void Reset();

  // All threads will be requested to finish.
  // It waits until all threads have finished.
  // This function must be called before saving the trajectory.
  void Shutdown();

  // Information from most recent processed frame
  // You can call this right after TrackMonocular (or stereo or RGBD)
  int GetTrackingState();
  std::vector<MapPoint*> GetTrackedMapPoints();
  std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

  // Save trajectory calculated
  void SaveTrajectory(const std::string &filename, const std::string &foldername);

  // Load saved trajectory
  bool LoadTrajectory(const std::string &filename);

  void save_as_tum(const std::string &filename);
  void save_scales(const std::string &filename);
  void save_tracking_state(const std::string &filename);

 private:
  // Input sensor
  eSensor mSensor;

  // Map structure that stores the pointers to all KeyFrames and MapPoints.
  Map* mpMap;

  // Tracker. It receives a frame and computes the associated camera pose.
  // It also decides when to insert a new keyframe, create some new MapPoints and
  // performs relocalization if tracking fails.
  Tracking* mpTracker;

  // Local Mapper. It manages the local map and performs local bundle adjustment.
  LocalMapping* mpLocalMapper;

  // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
  // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
  LoopClosing* mpLoopCloser;

  // System threads: Local Mapping, Loop Closing
  // The Tracking thread "lives" in the main execution thread that creates the System object.
  std::thread* mptLocalMapping;
  std::thread* mptLoopClosing;

  // Reset flag
  std::mutex mMutexReset;
  bool mbReset;

  // Change mode flags
  std::mutex mMutexMode;
  bool mbActivateLocalizationMode;
  bool mbDeactivateLocalizationMode;

  // Tracking state
  int mTrackingState;
  std::vector<MapPoint*> mTrackedMapPoints;
  std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
  std::mutex mMutexState;
  bool stopRequested_;          // True if stop is requested
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_SYSTEM_H
