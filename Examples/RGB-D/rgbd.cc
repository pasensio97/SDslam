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

bool LoadImages(const string &strAssociationFilename, vector<string> &vFilenamesRGB,
                vector<string> &vFilenamesD);

int main(int argc, char **argv) {
  vector<string> vFilenamesRGB;
  vector<string> vFilenamesD;
  cv::Mat im, imD;
  int nImages, ni = 0;
  bool useViewer = true;
  double freq = 1.0/30.0;
  std::string fname;

  if(argc != 4 && argc != 5) {
      cerr << endl << "Usage: ./rgbd path_to_settings path_to_sequence path_to_association [path_to_saved_map]" << endl;
      return 1;
  }

  // Read parameters
  SD_SLAM::Config &config = SD_SLAM::Config::GetInstance();
  if (!config.ReadParameters(argv[1])) {
    cerr << "[ERROR] Config file contains errors" << endl;
    return 1;
  }

  // Retrieve paths to images
  string strAssociationFilename = string(argv[3]);
  bool ok = LoadImages(strAssociationFilename, vFilenamesRGB, vFilenamesD);
  if (!ok) {
    cerr << "[ERROR] Couldn't find images, does " << strAssociationFilename << " exist?" << endl;
    return 1;
  }

  // Check consistency in the number of images and depthmaps
  nImages = vFilenamesRGB.size();
  if(vFilenamesRGB.empty()) {
      cerr << endl << "[ERROR] No images found in provided path." << endl;
      return 1;
  } else if(vFilenamesD.size()!=vFilenamesRGB.size()) {
      cerr << endl << "[ERROR] Different number of images for rgb and depth." << endl;
      return 1;
  }

  cout << "[INFO] Sequence has " << nImages << " images" << endl;

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  SD_SLAM::System SLAM(SD_SLAM::System::RGBD, true);

  // Check if a saved map is provided
  if (argc == 5) {
    SLAM.LoadTrajectory(string(argv[4]));
  }

#ifdef PANGOLIN
  // Create user interface
  SD_SLAM::Map * map = SLAM.GetMap();
  SD_SLAM::Tracking * tracker = SLAM.GetTracker();

  SD_SLAM::FrameDrawer * fdrawer = new SD_SLAM::FrameDrawer(map);
  SD_SLAM::MapDrawer * mdrawer = new SD_SLAM::MapDrawer(map);

  SD_SLAM::Viewer* viewer = nullptr;
  std::thread* tviewer = nullptr;

  if (useViewer) {
    viewer = new SD_SLAM::Viewer(&SLAM, fdrawer, mdrawer, tracker);
    tviewer = new std::thread(&SD_SLAM::Viewer::Run, viewer);
  }
#endif

  // Main loop
  while (ni<nImages && !SLAM.StopRequested()) {
    // Read image and depthmap from file
    cout << "[INFO] Reading Frame " << string(argv[2])+"/"+vFilenamesRGB[ni] << endl;
    fname = vFilenamesRGB[ni];
    im = cv::imread(string(argv[2])+"/"+vFilenamesRGB[ni], CV_LOAD_IMAGE_GRAYSCALE);
    imD = cv::imread(string(argv[2])+"/"+vFilenamesD[ni], CV_LOAD_IMAGE_UNCHANGED);

    if(im.empty()) {
      cerr << endl << "[ERROR] Failed to load image at: " << string(argv[2]) << "/" << vFilenamesRGB[ni] << endl;
      return 1;
    }

    SD_SLAM::Timer ttracking(true);

    // Pass the image to the SLAM system
    Eigen::Matrix4d pose = SLAM.TrackRGBD(im, imD, fname);

    // Set data to UI
#ifdef PANGOLIN
    fdrawer->Update(im, pose, tracker);
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
  SLAM.SaveTrajectory("trajectoryRGBD.yaml", "trajectoryRGBD");

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

bool LoadImages(const string &strAssociationFilename, vector<string> &vFilenamesRGB,
                vector<string> &vFilenamesD) {
  ifstream fAssociation;
  fAssociation.open(strAssociationFilename.c_str());
  if(!fAssociation.is_open())
    return false;

  while(!fAssociation.eof()) {
    string s;
    getline(fAssociation, s);
    if(!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      string sRGB, sD;
      ss >> t;
      ss >> sRGB;
      vFilenamesRGB.push_back(sRGB);
      ss >> t;
      ss >> sD;
      vFilenamesD.push_back(sD);
    }
  }

  return true;
}
