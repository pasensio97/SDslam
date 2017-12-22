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
#include <fstream>
#include <vector>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/cvaux.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vFilenames);

int main(int argc, char **argv) {
  bool debug = true;
  const cv::Size patternsize(6,4);  // Num internal corners of pattern
  const double size = 0.0302;  // Cell size (cm)

  vector<string> vFilenames;
  int nImages;
  cv::Mat cameraMatrix, distCoeffs;
  vector<cv::Mat> rvecs, tvecs;
  vector<vector<cv::Point2f>> coord2D;
  vector<vector<cv::Point3f>> coord3D;
  vector<cv::Point3f> corners3D;
  cv::Size cam_size;

  if(argc != 2) {
      cerr << endl << "Usage: ./calibration path_to_sequence" << endl;
      return 1;
  }

  // Retrieve paths to images
  string filename = string(argv[1])+"/files.txt";
  LoadImages(filename, vFilenames);
  nImages = vFilenames.size();
  cout << "Found " << nImages << " images" << endl;

  // Detect corners in images
  for(int i=0; i<nImages; i++) {
    vector<cv::Point2f> corners;
    stringstream ss;

    cout << "Reading image " << string(argv[1])+"/"+vFilenames[i] << endl;
    cv::Mat img = cv::imread(string(argv[1])+"/"+vFilenames[i], CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat img_color = cv::imread(string(argv[1])+"/"+vFilenames[i], CV_LOAD_IMAGE_COLOR);
    cam_size = img.size();

    bool found = cv::findChessboardCorners(img, patternsize, corners);

    if (found) {
      // Refine corners
      cv::cornerSubPix(img,corners,cv::Size(11,11),cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));
      coord2D.push_back(corners);

      if (debug)
        cv::drawChessboardCorners(img_color,patternsize,corners,found);
    }

    if (debug) {
      cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE );
      cv::imshow("Display window", img_color);
      cv::waitKey(0);
    }
  }

  // Set real coordinates
  for(int i=0; i<patternsize.height; i++)
    for(int j=0; j<patternsize.width; j++)
      corners3D.push_back(cv::Point3f(j*size, i*size, 0));
  coord3D.resize(coord2D.size(),corners3D);

  // Calculate intrinsics
  cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
  double rms = cv::calibrateCamera(coord3D, coord2D, cam_size, cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

  std::cout << "Camera Matrix:" << endl << cameraMatrix << std::endl;
  std::cout << "Distortion Vector: " << endl << distCoeffs << std::endl;
  std::cout << "Average error was: " << rms << std::endl;

  // Save undistorted images
  if (debug) {
    cv::Mat view, rview, map1, map2;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
      cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cam_size, 1, cam_size, 0),
      cam_size, CV_16SC2, map1, map2);

    for(int i=0; i<nImages; i++) {
      cout << "Saving image " << string(argv[1])+"/"+vFilenames[i] << endl;

      view = cv::imread(string(argv[1])+"/"+vFilenames[i], CV_LOAD_IMAGE_COLOR);
      cv::remap(view, rview, map1, map2, CV_INTER_LINEAR);
      cv::imwrite(string(argv[1])+"/"+vFilenames[i]+"_dst.png", rview);
    }
  }
 
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
