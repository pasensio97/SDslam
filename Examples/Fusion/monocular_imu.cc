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

bool LoadImages(const string &strFile, vector<string> &vFilenames);
void LoadIMU(const string &strFile, vector<vector<double>> &values);
void Associate(const vector<string> &vFilenames, const vector<vector<double>> &valuesIn, vector<int> &valuesIdx);

int main(int argc, char **argv) {
  vector<string> vFilenames;
  vector<vector<double>> vIMUValues;
  vector<int> vIdxValues;
  cv::Mat im;
  int nImages, nValues, nAssociated, ni = 0;
  bool useViewer = true;
  double freq = 1.0/30.0;

  if(argc != 4) {
    cerr << endl << "Usage: ./monocular_imu path_to_settings path_to_sequence path_to_IMU_data" << endl;
    return 1;
  }

  // Read parameters
  SD_SLAM::Config &config = SD_SLAM::Config::GetInstance();
  if (!config.ReadParameters(argv[1])) {
    cerr << "[ERROR] Config file contains errors" << endl;
    return 1;
  }

  // Retrieve paths to images
  string filename = string(argv[2])+"/files.txt";
  bool ok = LoadImages(filename, vFilenames);
  if (!ok) {
    cerr << "[ERROR] Couldn't find images, does " << filename << " exist?" << endl;
    return 1;
  }
  nImages = vFilenames.size();
  cout << "[INFO] Sequence has " << nImages << " images" << endl;

  // Read IMU values
  string filename_imu = string(argv[3]);
  LoadIMU(filename_imu, vIMUValues);
  nValues = vIMUValues.size();
  cout << "[INFO] Sequence has " << nValues << " IMU values" << endl;

  // Associate images and IMU values
  Associate(vFilenames, vIMUValues, vIdxValues);
  nAssociated = vIdxValues.size();
  cout << "[INFO] Associated " << nAssociated << " values" << endl;

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  SD_SLAM::System SLAM(SD_SLAM::System::MONOCULAR_IMU, true);

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
  while (ni<nImages && !SLAM.StopRequested()) {
    // Read image from file
    cout << "[INFO] Reading Frame " << string(argv[2])+"/"+vFilenames[ni] << endl;
    im = cv::imread(string(argv[2])+"/"+vFilenames[ni], CV_LOAD_IMAGE_GRAYSCALE);

    if(im.empty()) {
      cerr << endl << "[ERROR] Failed to load image at: "  << string(argv[2]) << "/" << vFilenames[ni] << endl;
      return 1;
    }

    // Get values
    vector<double> values = vIMUValues[vIdxValues[ni]];
    cout << "[INFO] Reading IMU values for timestamp "  << (long) values[0] << endl;
    values.erase(values.begin()); // Remove timestamp

    SD_SLAM::Timer ttracking(true);

    // Pass the image and measurements to the SLAM system
    Eigen::Matrix4d pose = SLAM.TrackFusion(im, values);

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

bool LoadImages(const string &strFile, vector<string> &vFilenames) {
  ifstream f;
  f.open(strFile.c_str());
  if(!f.is_open())
    return false;

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

  return true;
}

void LoadIMU(const string &strFile, vector<vector<double>> &values) {
  ifstream f;
  f.open(strFile.c_str());
  while(!f.eof()) {
    string s;
    getline(f, s);
    if(!s.empty()) {
      if (s[0] == '#') {
        continue; // skip comments
      }

      stringstream ss;
      long ts;
      double wx, wy, wz, ax, ay, az;
      vector<double> cvalues;

      std::replace(s.begin(), s.end(), ',', ' ');
      ss << s;
      ss >> ts;
      ss >> wx;
      ss >> wy;
      ss >> wz;
      ss >> ax;
      ss >> ay;
      ss >> az;
      cvalues.push_back((double)ts);
      cvalues.push_back(wx);
      cvalues.push_back(wy);
      cvalues.push_back(wz);
      cvalues.push_back(ax);
      cvalues.push_back(ay);
      cvalues.push_back(az);
      values.push_back(cvalues);
    }
  }
}

void Associate(const vector<string> &vFilenames, const vector<vector<double>> &values, vector<int> &valuesIdx) {
  int cindex = 0;
  int vsize = values.size();

  for (auto it=vFilenames.begin(); it!=vFilenames.end(); it++) {

    // Get time stamp
    size_t lastindex = (*it).find_last_of(".");
    string rawname =  (*it).substr(0, lastindex);
    string::size_type sz;
    long ts = stol(rawname,&sz);

    // Search in imu values
    bool found = false;
    for (int i=cindex; i<vsize && !found; i++) {
      long cts = values[i][0];

      // Save equal or great
      if (cts >= ts) {
        cindex = i;
        found = true;
      }
    }

    // Save last one
    if (!found)
      cindex = vsize-1;

    // Save index
    valuesIdx.push_back(cindex);
  }

}
