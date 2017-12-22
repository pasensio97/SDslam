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

#include <opencv2/core/core.hpp>
#include "Config.h"
#include "extra/log.h"

using std::endl;

namespace SD_SLAM {

Config::Config() {
  // Default values
  camera_params_.w = 640;
  camera_params_.h = 480;
  camera_params_.fx = 500.0;
  camera_params_.fy = 500.0;
  camera_params_.cx = 320.0;
  camera_params_.cy = 240.0;
  camera_params_.k1 = 0.0;
  camera_params_.k2 = 0.0;
  camera_params_.p1 = 0.0;
  camera_params_.p2 = 0.0;
  camera_params_.k3 = 0.0;
  camera_params_.fps = 30.0;
  camera_params_.bf = 40.0;

  kUsePattern_ = false;
  kThDepth_ = 40.0;
  kDepthMapFactor_ = 5000.0;

  kNumFeatures_ = 1000;
  kScaleFactor_ = 2.0;
  kNumLevels_ = 5;
  kThresholdFAST_ = 20;

  kKeyFrameSize_ = 0.05;
  kKeyFrameLineWidth_ = 1.0;
  kGraphLineWidth_ = 0.9;
  kPointSize_ = 2.0;
  kCameraSize_ = 0.08;
  kCameraLineWidth_ = 3.0;
  kViewpointX_ = 0.0;
  kViewpointY_ = -0.7;
  kViewpointZ_ = -1.8;
  kViewpointF_ = 500.0;
}

bool Config::ReadParameters(std::string filename) {
  cv::FileStorage fs;

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

  // Camera
  if (fs["Camera.Width"].isNamed()) fs["Camera.Width"] >> camera_params_.w;
  if (fs["Camera.Height"].isNamed()) fs["Camera.Height"] >> camera_params_.h;
  if (fs["Camera.fx"].isNamed()) fs["Camera.fx"] >> camera_params_.fx;
  if (fs["Camera.fy"].isNamed()) fs["Camera.fy"] >> camera_params_.fy;
  if (fs["Camera.cx"].isNamed()) fs["Camera.cx"] >> camera_params_.cx;
  if (fs["Camera.cy"].isNamed()) fs["Camera.cy"] >> camera_params_.cy;
  if (fs["Camera.k1"].isNamed()) fs["Camera.k1"] >> camera_params_.k1;
  if (fs["Camera.k2"].isNamed()) fs["Camera.k2"] >> camera_params_.k2;
  if (fs["Camera.p1"].isNamed()) fs["Camera.p1"] >> camera_params_.p1;
  if (fs["Camera.p2"].isNamed()) fs["Camera.p2"] >> camera_params_.p2;
  if (fs["Camera.k3"].isNamed()) fs["Camera.k3"] >> camera_params_.k3;
  if (fs["Camera.fps"].isNamed()) fs["Camera.fps"] >> camera_params_.fps;
  if (fs["Camera.bf"].isNamed()) fs["Camera.bf"] >> camera_params_.bf;

  if (fs["UsePattern"].isNamed()) fs["UsePattern"] >> kUsePattern_;
  if (fs["ThDepth"].isNamed()) fs["ThDepth"] >> kThDepth_;
  if (fs["DepthMapFactor"].isNamed()) fs["DepthMapFactor"] >> kDepthMapFactor_;

  // ORB Extractor
  if (fs["ORBextractor.nFeatures"].isNamed()) fs["ORBextractor.nFeatures"] >> kNumFeatures_;
  if (fs["ORBextractor.scaleFactor"].isNamed()) fs["ORBextractor.scaleFactor"] >> kScaleFactor_;
  if (fs["ORBextractor.nLevels"].isNamed()) fs["ORBextractor.nLevels"] >> kNumLevels_;
  if (fs["ORBextractor.thresholdFAST"].isNamed()) fs["ORBextractor.thresholdFAST"] >> kThresholdFAST_;

  // UI
  if (fs["Viewer.KeyFrameSize"].isNamed()) fs["Viewer.KeyFrameSize"] >> kKeyFrameSize_;
  if (fs["Viewer.KeyFrameLineWidth"].isNamed()) fs["Viewer.KeyFrameLineWidth"] >> kKeyFrameLineWidth_;
  if (fs["Viewer.GraphLineWidth"].isNamed()) fs["Viewer.GraphLineWidth"] >> kGraphLineWidth_;
  if (fs["Viewer.PointSize"].isNamed()) fs["Viewer.PointSize"] >> kPointSize_;
  if (fs["Viewer.CameraSize"].isNamed()) fs["Viewer.CameraSize"] >> kCameraSize_;
  if (fs["Viewer.CameraLineWidth"].isNamed()) fs["Viewer.CameraLineWidth"] >> kCameraLineWidth_;
  if (fs["Viewer.ViewpointX"].isNamed()) fs["Viewer.ViewpointX"] >> kViewpointX_;
  if (fs["Viewer.ViewpointY"].isNamed()) fs["Viewer.ViewpointY"] >> kViewpointY_;
  if (fs["Viewer.ViewpointZ"].isNamed()) fs["Viewer.ViewpointZ"] >> kViewpointZ_;
  if (fs["Viewer.ViewpointF"].isNamed()) fs["Viewer.ViewpointF"] >> kViewpointF_;
  
  fs.release();

  return true;
}

void Config::SetCameraIntrinsics(double w, double h, double fx, double fy, double cx, double cy) {
  camera_params_.w = w;
  camera_params_.h = h;
  camera_params_.fx = fx;
  camera_params_.fy = fy;
  camera_params_.cx = cx;
  camera_params_.cy = cy;
}

void Config::SetCameraDistortion(double k1, double k2, double p1, double p2, double k3) {
  camera_params_.k1 = k1;
  camera_params_.k2 = k2;
  camera_params_.p1 = p1;
  camera_params_.p2 = p2;
  camera_params_.k3 = k3;
}

}  // namespace SD_SLAM
