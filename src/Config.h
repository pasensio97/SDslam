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
  int w;
  int h;
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

  // Set parameters
  void SetCameraIntrinsics(double w, double h, double fx, double fy, double cx, double cy);
  void SetCameraDistortion(double k1, double k2, double p1, double p2, double k3);
  void SetUsePattern(bool use_pattern);

  // Get parameters
  static double Width() { return GetInstance().camera_params_.w; }
  static double Height() { return GetInstance().camera_params_.h; }
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

  static double UsePattern() { return GetInstance().kUsePattern_; }
  static double ThDepth() { return GetInstance().kThDepth_; }
  static double DepthMapFactor() { return GetInstance().kDepthMapFactor_; }

  static int NumFeatures() { return GetInstance().kNumFeatures_; }
  static double ScaleFactor() { return GetInstance().kScaleFactor_; }
  static int NumLevels() { return GetInstance().kNumLevels_; }
  static int ThresholdFAST() { return GetInstance().kThresholdFAST_; }

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

  static std::string CameraTopic() { return GetInstance().kCameraTopic_; }
  static std::string DepthTopic() { return GetInstance().kDepthTopic_; }
  static std::string IMUTopic() { return GetInstance().kIMUTopic_; }
  static std::string BaseFrame() { return GetInstance().kBaseFrame_; }
  static std::string CameraFrame() { return GetInstance().kCameraFrame_; }
  static bool UseImagesTimeStamps() { return GetInstance().kUseImagesTimeStamps_; }

  static double MadgwickGain() { return GetInstance().kMadgwickGain_; }
  static cv::Mat RotationIMUToCam() { return GetInstance().kRotIMUToCam_; }
  static double ScaleUpdateFactor() { return GetInstance().kIMUScaleUpdateFactor_; }

 private:
  Config();

  // Camera
  CameraParameters camera_params_;

  bool kUsePattern_;
  double kThDepth_;
  double kDepthMapFactor_;

  // ORB Extractor
  int kNumFeatures_;
  double kScaleFactor_;
  int kNumLevels_;
  int kThresholdFAST_;

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

  // ROS
  std::string kCameraTopic_;
  std::string kDepthTopic_;
  std::string kIMUTopic_;
  std::string kBaseFrame_;
  std::string kCameraFrame_;
  bool kUseImagesTimeStamps_;

  // IMU CONFIGURATION
  double kMadgwickGain_;
  cv::Mat kRotIMUToCam_;
  double kIMUScaleUpdateFactor_;

};

}  // namespace SD_SLAM


#endif  // SD_SLAM_CONFIG_H_
