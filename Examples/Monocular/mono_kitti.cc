/**
 *
 *  Copyright (C) 2017 Eduardo Perdices <eperdices at gsyc dot es>
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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include "System.h"
#include "Tracking.h"
#include "Map.h"
#include "Config.h"
#include "extra/timer.h"
#include "Examples/Help/kittiRawReader.h"
#include "ui/Viewer.h"
#include "ui/FrameDrawer.h"
#include "ui/MapDrawer.h"

using namespace std;


void ShowPose(const Eigen::Matrix4d &pose) {
  Eigen::Matrix4d wpose;
  wpose.setIdentity();
  wpose.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0).transpose();
  wpose.block<3, 1>(0, 3) = -pose.block<3, 3>(0, 0).transpose()*pose.block<3, 1>(0, 3);

  Eigen::Quaterniond q(wpose.block<3, 3>(0, 0));
  cout << "[INFO] World pose: [" << wpose(0, 3) << " " << wpose(1, 3) << " " << wpose(2, 3) << "]";
  cout << "[" << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "]" << endl;
}


int main(int argc, char **argv) {
  if(argc != 4){
    cerr << endl << "Usage: ./mono_kitti dataset_path sequence config_yaml" << endl;
    return 1;
  }

  string dataset_path = argv[1];
  string sequence = argv[2];
  // Read parameters
  SD_SLAM::Config &config = SD_SLAM::Config::GetInstance();
  if (!config.ReadParameters(argv[3])) {
    cerr << "[ERROR] Config file contains errors" << endl;
    return 1;
  }

  KittiRawReader kitti(dataset_path, sequence);
  std::vector<double> timestamps = kitti.load_timestamps();
  std::vector<std::string> image_filenames = kitti.load_left_images();

  double dt = 0.1;
  int ni = 0;
  int n_images = image_filenames.size();
  cv::Mat image;

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  SD_SLAM::System SLAM(SD_SLAM::System::MONOCULAR, true);

  // Create user interface
  SD_SLAM::Map * map = SLAM.GetMap();
  SD_SLAM::Tracking * tracker = SLAM.GetTracker();

  SD_SLAM::FrameDrawer * fdrawer = new SD_SLAM::FrameDrawer(map);
  SD_SLAM::MapDrawer * mdrawer = new SD_SLAM::MapDrawer(map);

  SD_SLAM::Viewer* viewer = nullptr;
  std::thread* tviewer = nullptr;

  viewer = new SD_SLAM::Viewer(&SLAM, fdrawer, mdrawer);
  tviewer = new std::thread(&SD_SLAM::Viewer::Run, viewer);

  // Main loop
  while (ni < n_images && !SLAM.StopRequested()) {
    printf("[%i/%i]\n", ni, n_images);
    image = kitti.get_image(image_filenames[ni]);

    SD_SLAM::Timer ttracking(true);

    // Pass the image to the SLAM system
    Eigen::Matrix4d pose = SLAM.TrackMonocular(image, "");

    // Show world pose
    ShowPose(pose);

    // Set data to UI
    fdrawer->Update(image, pose, tracker);
    mdrawer->SetCurrentCameraPose(pose);

    ttracking.Stop();
    double delay = ttracking.GetTime();

    // Wait to load the next frame
    if(delay < dt)
      usleep((dt-delay)*1e6);

    if (viewer->isFinished())
      return 0;

    ni++;
  }
  

  // Stop all threads
  SLAM.Shutdown();

  viewer->RequestFinish();
  while (!viewer->isFinished())
    usleep(5000);

  tviewer->join();

  return 0;
}
