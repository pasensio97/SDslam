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
#ifdef PANGOLIN
#include "ui/Viewer.h"
#include "ui/FrameDrawer.h"
#include "ui/MapDrawer.h"
#endif

using namespace std;

void LoadImages(const string &strFile, vector<string> &vFilenames);

int main(int argc, char **argv) {
  vector<string> vFilenames;
  cv::Mat im;
  cv::VideoCapture * cap = nullptr;
  int nImages, ni = 0;
  bool useViewer = true;
  bool live = false;
  double freq = 1.0/30.0;
  std::string src = "";

  if(argc != 3) {
      cerr << endl << "Usage: ./monocular path_to_settings path_to_sequence/device_number" << endl;
      return 1;
  }

  // Read parameters
  SD_SLAM::Config &config = SD_SLAM::Config::GetInstance();
  if (!config.ReadParameters(argv[1])) {
    cerr << "[ERROR] Config file contains errors" << endl;
    return 1;
  }

  // Check if live mode is activated
  if (isdigit(argv[2][0])) {
    live = true;
    string sdevice = string(argv[2]);
    int device;
    istringstream(sdevice) >> device;

    cap = new cv::VideoCapture(device);
    cap->set(CV_CAP_PROP_FRAME_WIDTH, config.Width());
    cap->set(CV_CAP_PROP_FRAME_HEIGHT, config.Height());
    cap->set(CV_CAP_PROP_FPS, config.fps());

    if (!cap->isOpened()) {
      cerr << "[ERROR] Couldn't open video device" << endl;
      return 1;
    }

    std::cout << "[INFO] Get images from /dev/video" << device << std::endl;
    nImages = INT_MAX;
  } else {
    // Retrieve paths to images
    string filename = string(argv[2])+"/files.txt";
    LoadImages(filename, vFilenames);
    nImages = vFilenames.size();
    cout << "[INFO] Sequence has " << nImages << " images" << endl;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  SD_SLAM::System SLAM(SD_SLAM::System::MONOCULAR, true);

#ifdef PANGOLIN
  // Create user interface
  SD_SLAM::Map * map = SLAM.GetMap();
  SD_SLAM::Tracking * tracker = SLAM.GetTracker();

  SD_SLAM::FrameDrawer * fdrawer = new SD_SLAM::FrameDrawer(map);
  SD_SLAM::MapDrawer * mdrawer = new SD_SLAM::MapDrawer(map);

  SD_SLAM::Viewer* viewer = nullptr;
  std::thread* tviewer = nullptr;

  if (useViewer) {
    viewer = new SD_SLAM::Viewer(&SLAM, fdrawer, mdrawer);
    tviewer = new std::thread(&SD_SLAM::Viewer::Run, viewer);
  }
#endif

  // Main loop
  while (ni<nImages) {
    if (live) {
      cv::Mat frame;
      *cap >> frame;
      cv::cvtColor(frame, im, CV_RGB2GRAY);
    } else {
      // Read image from file
      src = string(argv[2])+"/"+vFilenames[ni];
      cout << "[INFO] Reading Frame " << src << endl;
      im = cv::imread(src, CV_LOAD_IMAGE_GRAYSCALE);

      if(im.empty()) {
        cerr << endl << "[ERROR] Failed to load image at: "  << string(argv[2]) << "/" << vFilenames[ni] << endl;
        return 1;
      }
    }

    SD_SLAM::Timer ttracking(true);

    // Pass the image to the SLAM system
    Eigen::Matrix4d pose = SLAM.TrackMonocular(im, src);

    // Set data to UI
#ifdef PANGOLIN
    fdrawer->Update(tracker);
    mdrawer->SetCurrentCameraPose(pose);
#endif

    ttracking.Stop();
    double delay = ttracking.GetTime();

    // Wait to load the next frame
    if(delay<freq)
      usleep((freq-delay)*1e6);

#ifdef PANGOLIN
    if (useViewer && viewer->isFinished())
      return 0;
#endif

    ni++;
  }

  // Stop all threads
  SLAM.Shutdown();

  // Save data
  SLAM.SaveTrajectory("trajectory.json");

#ifdef PANGOLIN
  if (useViewer) {
    viewer->RequestFinish();
    while (!viewer->isFinished())
      usleep(5000);

    tviewer->join();
  }
#endif

  return 0;
}

void LoadImages(const string &strFile, vector<string> &vFilenames) {
  ifstream f;
  f.open(strFile.c_str());
  while(!f.eof()) {
    string s;
    getline(f, s);
    if(!s.empty()) {
      stringstream ss;
      string sRGB;
      ss << s;
      ss >> sRGB;
      vFilenames.push_back(sRGB);
    }
  }
}
