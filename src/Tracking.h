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

#ifndef SD_SLAM_TRACKING_H
#define SD_SLAM_TRACKING_H

#include <mutex>
#include <string>
#include <list>
#include <vector>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "PatternDetector.h"
#include "System.h"
#include "sensors/EKF.h"
#include "inertial/IMU_Measurements.h"
#include "inertial/attitude_estimators/Madgwick.h"
#include "inertial/PredictionModels.h"
#include "inertial/ScaleInitializer.h"

namespace SD_SLAM {

class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking {
 public:
  // Tracking states
  enum eTrackingState{
    SYSTEM_NOT_READY = -1,
    NO_IMAGES_YET = 0,
    NOT_INITIALIZED = 1,
    OK = 2,
    LOST = 3,
    OK_IMU = 4
  };

 public:
  Tracking(System* pSys, Map* pMap, const int sensor);

  // Preprocess the input and call Track(). Extract features and performs sterFeo matching.
  Eigen::Matrix4d GrabImageRGBD(const cv::Mat &im, const cv::Mat &imD, const std::string filename);
  Eigen::Matrix4d GrabImageMonocular(const cv::Mat &im, const std::string filename);
  Eigen::Matrix4d GrabImageFusion(const cv::Mat &im, const double dt); // Temporal used to pass dt

  // Create new frame and extract features
  Frame CreateFrame(const cv::Mat &im);
  Frame CreateFrame(const cv::Mat &im, const cv::Mat &imD);

  inline void SetLocalMapper(LocalMapping* pLocalMapper) {
    mpLocalMapper = pLocalMapper;
  }

  inline void SetLoopClosing(LoopClosing* pLoopClosing) {
    mpLoopClosing = pLoopClosing;
  }

  inline void SetMeasurements(const std::vector<double> &measurements) {
    measurements_ = measurements;
  }

  inline void SetIMUMeasurements(const IMU_Measurements &measurements) {
    imu_measurements_ = measurements;
  }

  inline void SetReferenceKeyFrame(KeyFrame * kf) {
    mpReferenceKF = kf;
  }

  inline float GetDepthFactor() const { return mDepthMapFactor; }

  inline bool OnlyTracking() const { return mbOnlyTracking; }

  inline eTrackingState GetState() { return mState; }
  inline eTrackingState GetLastState() { return mLastProcessedState; }
  inline void ForceRelocalization() { mState = LOST; }

  inline Frame& GetCurrentFrame() { return mCurrentFrame; }
  inline Frame& GetInitialFrame() { return mInitialFrame; }

  inline std::vector<int> GetInitialMatches() { return mvIniMatches; }

  void Reset();
  
  void PatternCellSize(double w, double h);


  // Use this function if you have deactivated local mapping and you only want to localize the camera.
  void InformOnlyTracking(const bool &flag);

 protected:
  // Main tracking function. It is independent of the input sensor.
  void Track();
  void Track_test();
  void Track_IMU_Reloc();
  void Track_IMU_Reinit();

  // Map initialization for stereo and RGB-D
  void StereoInitialization();

  // Map initialization for monocular
  void MonocularInitialization();
  void MonocularInitializationIMU();

  void MonocularReInitializationIMU(const Matrix4d & curr_imu_prediction);
  void CreateInitialMapMonocular();

  // Initialization with pattern
  void PatternInitialization();

  void CheckReplacedInLastFrame();
  bool TrackReferenceKeyFrame();
  void UpdateLastFrame();
  bool TrackWithMotionModel();
  bool TrackWithNewIMUModel(); // in dev
  bool TrackVisual(Eigen::Matrix4d predicted_pose); // code cte in TrackWithNewIMUModel
  int TrackMMVisual(Frame &frame, bool & use_align_image); // code cte in TrackWithNewIMUModel

  bool Relocalization();
  bool RelocalizationIMU();
  
  void UpdateLocalMap();
  void UpdateLocalPoints();
  void UpdateLocalKeyFrames();

  bool TrackLocalMap();
  void SearchLocalPoints();

  bool NeedNewKeyFrame();
  bool NeedNewKeyFrame_test();
  void CreateNewKeyFrame(bool is_fake = false);

  // Other Thread Pointers
  LocalMapping* mpLocalMapper;
  LoopClosing* mpLoopClosing;

  // Tracking state
  eTrackingState mState;
  eTrackingState mLastProcessedState;

  // Input sensor
  int mSensor;

  // Current Frame
  Frame mCurrentFrame;

  // ORB
  ORBextractor* mpORBextractorLeft;
  ORBextractor* mpIniORBextractor;

  // Initalization (only for monocular)
  Initializer* mpInitializer;
  PatternDetector mpPatternDetector;

  // Local Map
  KeyFrame* mpReferenceKF;
  std::vector<KeyFrame*> mvpLocalKeyFrames;
  std::vector<MapPoint*> mvpLocalMapPoints;

  // System
  System* mpSystem;

  // Map
  Map* mpMap;

  // Calibration matrix
  Eigen::Matrix3d mK;
  cv::Mat mDistCoef;
  float mbf;

  // New KeyFrame rules (according to fps)
  int mMinFrames;
  int mMaxFrames;

  // Threshold close/far points
  // Points seen as close by the stereo/RGBD sensor are considered reliable
  // and inserted from just one frame. Far points requiere a match in two keyframes.
  float mThDepth;

  // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
  float mDepthMapFactor;

  // Current matches in frame
  int mnMatchesInliers;

  // Last Frame, KeyFrame and Relocalisation Info
  KeyFrame* mpLastKeyFrame;
  Frame mLastFrame;
  unsigned int mnLastKeyFrameId;
  unsigned int mnLastRelocFrameId;

  // Sensor model
  EKF* motion_model_;
  std::vector<double> measurements_;
  IMU_Measurements imu_measurements_;

  std::list<MapPoint*> mlpTemporalPoints;
  int threshold_;

  // Initialization Variables (Monocular)
  std::vector<int> mvIniLastMatches;
  std::vector<int> mvIniMatches;
  std::vector<cv::Point2f> mvbPrevMatched;
  std::vector<cv::Point3f> mvIniP3D;
  Frame mInitialFrame;

  // Save last relative pose
  Eigen::Matrix4d lastRelativePose_;

  // True if local mapping is deactivated and we are performing only localization
  bool mbOnlyTracking;

  bool usePattern;

  // Image align
  bool align_image_;


 public:
  // Inertial
  double curr_timestamp = 0.0;
  double dt_;

  Frame last_valid_frame;
  Matrix4d last_valid_wpose;
  Map* _new_map;
  Frame newInitFrame, newCurrFrame;
  int nKFs_on_reinit = 0;
  
  Matrix4d curr_imu_prediction;
  new_IMU_model new_imu_model;
  Matrix4d mIniPoseInvIMU;
  int _frames_last_fake_kf = 0;

  bool used_imu_model = false;

  void estimate_scale(KeyFrame* curr_kf, KeyFrame* last_kf);
  KeyFrame* mpFirstReloadKeyFrame = nullptr;

  // Use BA on Reinit
  bool _use_BA_on_reinit = true;
  double _angle_th = 25.0;

  // Hybrid motion model 
  bool _use_hybrid_model = false;
  uint _kf_to_use_hm = 20;
  bool _align_image_inertial = false;
  bool _full_hyb_mode = false;
  vector<int> _hyb_matches;  // Test var: Save n_matches_visual, n_matches_inertial, visual_time, inertial_time
  vector<double> _hyb_times;  // Test var: Save  visual_time, inertial_time

  // Scale model
  bool update_scale = true;
  double _scale_update_factor;  //update factor 
  ScaleInitializer::eModel _scale_model = ScaleInitializer::DIRECT_SINGLE;
  ScaleInitializer _scale_initializer = ScaleInitializer(_scale_model, 1, 0); // min 10

  // Test
  int first_proj, second_proj, inliers_on_pred, inliers_on_localmap;
  bool stay_in_curve; 

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_TRACKING_H
