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

#ifndef SD_SLAM_CONFIG_H_
#define SD_SLAM_CONFIG_H_

#include <iostream>
#include <string>

namespace SD_SLAM {

struct CameraParameters {
  double fx;
  double fy;
  double cx;
  double cy;
  double k1;
  double k2;
  double p1;
  double p2;
  double k3;
  double fps;
  double bf;
};

class Config {
 public:
  // Singleton
  static Config& GetInstance() {
    static Config instance;
    return instance;
  }

  // Read parameters from file
  bool ReadParameters(std::string filename);

  static double fx() { return GetInstance().camera_params_.fx; }
  static double fy() { return GetInstance().camera_params_.fy; }
  static double cx() { return GetInstance().camera_params_.cx; }
  static double cy() { return GetInstance().camera_params_.cy; }
  static double k1() { return GetInstance().camera_params_.k1; }
  static double k2() { return GetInstance().camera_params_.k2; }
  static double p1() { return GetInstance().camera_params_.p1; }
  static double p2() { return GetInstance().camera_params_.p2; }
  static double k3() { return GetInstance().camera_params_.k3; }
  static double fps() { return GetInstance().camera_params_.fps; }
  static double bf() { return GetInstance().camera_params_.bf; }

  static double ThDepth() { return GetInstance().kThDepth_; }
  static double DepthMapFactor() { return GetInstance().kDepthMapFactor_; }

  static double KeyFrameSize() { return GetInstance().kKeyFrameSize_; }
  static double KeyFrameLineWidth() { return GetInstance().kKeyFrameLineWidth_; }
  static double GraphLineWidth() { return GetInstance().kGraphLineWidth_; }
  static double PointSize() { return GetInstance().kPointSize_; }
  static double CameraSize() { return GetInstance().kCameraSize_; }
  static double CameraLineWidth() { return GetInstance().kCameraLineWidth_; }
  static double ViewpointX() { return GetInstance().kViewpointX_; }
  static double ViewpointY() { return GetInstance().kViewpointY_; }
  static double ViewpointZ() { return GetInstance().kViewpointZ_; }
  static double ViewpointF() { return GetInstance().kViewpointF_; }

  static int NumFeatures() { return GetInstance().kNumFeatures_; }
  static double ScaleFactor() { return GetInstance().kScaleFactor_; }
  static int NumLevels() { return GetInstance().kNumLevels_; }
  static int IniThFAST() { return GetInstance().kIniThFAST_; }
  static int MinThFAST() { return GetInstance().kMinThFAST_; }

 private:
  Config();

  // Camera
  CameraParameters camera_params_;

  double kThDepth_;
  double kDepthMapFactor_;

  // ORB Extractor
  int kNumFeatures_;
  double kScaleFactor_;
  int kNumLevels_;
  int kIniThFAST_;
  int kMinThFAST_;

  // UI
  double kKeyFrameSize_;
  double kKeyFrameLineWidth_;
  double kGraphLineWidth_;
  double kPointSize_;
  double kCameraSize_;
  double kCameraLineWidth_;
  double kViewpointX_;
  double kViewpointY_;
  double kViewpointZ_;
  double kViewpointF_;
};

}  // namespace SD_SLAM


#endif  // SD_SLAM_CONFIG_H_
