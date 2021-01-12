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

#include "Tracking.h"
#include <iostream>
#include <mutex>
#include <unistd.h>
#include "ORBmatcher.h"
#include "Converter.h"
#include "Optimizer.h"
#include "ImageAlign.h"
#include "Config.h"
#include "extra/log.h"
#include "sensors/ConstantVelocity.h"
#include "sensors/IMU.h"
#include <math.h>
#include "inertial/tools/Estimator.h"

using namespace std;

namespace SD_SLAM {

Tracking::Tracking(System *pSys, Map *pMap, const int sensor):
  mState(NO_IMAGES_YET), mSensor(sensor), mpInitializer(static_cast<Initializer*>(NULL)),
  mpPatternDetector(), mpSystem(pSys), mpMap(pMap), mnLastRelocFrameId(0), mbOnlyTracking(false),
 new_imu_model(new_IMU_model(1, true, Config::MadgwickGain())), _scale_update_factor(Config::ScaleUpdateFactor())
{
  // Load camera parameters
  float fx = Config::fx();
  float fy = Config::fy();
  float cx = Config::cx();
  float cy = Config::cy();

  mK.setIdentity();
  mK(0, 0) = fx;
  mK(1, 1) = fy;
  mK(0, 2) = cx;
  mK(1, 2) = cy;

  cv::Mat DistCoef(4, 1, CV_32F);
  DistCoef.at<float>(0) = Config::k1();
  DistCoef.at<float>(1) = Config::k2();
  DistCoef.at<float>(2) = Config::p1();
  DistCoef.at<float>(3) = Config::p2();
  const float k3 = Config::k3();
  if (k3 != 0) {
    DistCoef.resize(5);
    DistCoef.at<float>(4) = k3;
  }
  DistCoef.copyTo(mDistCoef);

  mbf = Config::bf();

  float fps = Config::fps();
  if (fps == 0)
    fps=30;

  // Max/Min Frames to insert keyframes and to check relocalisation
  mMinFrames = 0;
  mMaxFrames = fps;

  cout << endl << "Camera Parameters: " << endl;
  cout << "- fx: " << fx << endl;
  cout << "- fy: " << fy << endl;
  cout << "- cx: " << cx << endl;
  cout << "- cy: " << cy << endl;
  cout << "- k1: " << DistCoef.at<float>(0) << endl;
  cout << "- k2: " << DistCoef.at<float>(1) << endl;
  if (DistCoef.rows==5)
    cout << "- k3: " << DistCoef.at<float>(4) << endl;
  cout << "- p1: " << DistCoef.at<float>(2) << endl;
  cout << "- p2: " << DistCoef.at<float>(3) << endl;
  cout << "- fps: " << fps << endl;

  // Load ORB parameters
  int nFeatures = Config::NumFeatures();
  float fScaleFactor = Config::ScaleFactor();
  int nLevels = Config::NumLevels();
  int fThFAST = Config::ThresholdFAST();

  mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor,nLevels, fThFAST);

  if (sensor!=System::RGBD)
    mpIniORBextractor = new ORBextractor(2*nFeatures, fScaleFactor,nLevels, fThFAST);

  cout << endl  << "ORB Extractor Parameters: " << endl;
  cout << "- Number of Features: " << nFeatures << endl;
  cout << "- Scale Levels: " << nLevels << endl;
  cout << "- Scale Factor: " << fScaleFactor << endl;
  cout << "- Fast Threshold: " << fThFAST << endl;

  if (sensor==System::RGBD) {
    mThDepth = mbf*(float)Config::ThDepth()/fx;
    cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;

    mDepthMapFactor = Config::DepthMapFactor();
    if (fabs(mDepthMapFactor)<1e-5)
      mDepthMapFactor=1;
    else
      mDepthMapFactor = 1.0f/mDepthMapFactor;
  }

  if (sensor==System::MONOCULAR_IMU_NEW){
    cout << endl  << "Inertial Parameters: " << endl;
    cout << "- Madgwick gain: " << Config::MadgwickGain() << endl;
    Matrix3d rot = Converter::toMatrix3d(Config::RotationIMUToCam());
    new_imu_model.set_rotation_imu_to_world(rot);
    cout << "- Rotation IMU to World:\n" << rot << endl;
  }

  threshold_ = 8;  // 8
  usePattern = Config::UsePattern();
  align_image_ = true;

  if (usePattern)
    std::cout << "Use pattern for initialization" << std::endl;

  mpLoopClosing = nullptr;
  mpLocalMapper = nullptr;

  lastRelativePose_.setZero();

  // Set motion model
  Sensor * sensor_model;
  if (sensor == System::MONOCULAR_IMU)
    sensor_model = new IMU();
  else
    sensor_model = new ConstantVelocity();
  motion_model_ = new EKF(sensor_model);
}

Eigen::Matrix4d Tracking::GrabImageRGBD(const cv::Mat &im, const cv::Mat &imD, const std::string filename) {
  cv::Mat imDepth = imD;

  // Image must be in gray scale
  assert(im.channels() == 1);

  if ((fabs(mDepthMapFactor-1.0f) > 1e-5) || imD.type() != CV_32F)
    imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

  mCurrentFrame = Frame(im, imDepth, mpORBextractorLeft, mK, mDistCoef, mbf, mThDepth);

  Track();

  return mCurrentFrame.GetPose();
}


Eigen::Matrix4d Tracking::GrabImageMonocular(const cv::Mat &im, const std::string filename) {
  // Image must be in gray scale
  assert(im.channels() == 1);

  if (mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
    mCurrentFrame = Frame(im, mpIniORBextractor, mK, mDistCoef, mbf, mThDepth);
  else
    mCurrentFrame = Frame(im, mpORBextractorLeft, mK, mDistCoef, mbf, mThDepth);

  Track();

  return mCurrentFrame.GetPose();
}

Eigen::Matrix4d Tracking::GrabImageFusion(const cv::Mat &im, const double dt) {
  // Image must be in gray scale 8UB
  // Img + Imu
  assert(im.channels() == 1);
  dt_ = dt;

  if (mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
    mCurrentFrame = Frame(im, mpIniORBextractor, mK, mDistCoef, mbf, mThDepth, curr_timestamp);
  else
    mCurrentFrame = Frame(im, mpORBextractorLeft, mK, mDistCoef, mbf, mThDepth,curr_timestamp);

  Track_IMU_Reinit();
  return mCurrentFrame.GetPose();
}

Frame Tracking::CreateFrame(const cv::Mat &im) {
  return Frame(im, mpORBextractorLeft, mK, mDistCoef, mbf, mThDepth);
}

Frame Tracking::CreateFrame(const cv::Mat &im, const cv::Mat &imD) {
  cv::Mat imDepth = imD;

  if ((fabs(mDepthMapFactor-1.0f) > 1e-5) || imD.type() != CV_32F)
    imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

  return Frame(im, imDepth, mpORBextractorLeft, mK, mDistCoef, mbf, mThDepth);
}

void Tracking::Track() {
  if (mState==NO_IMAGES_YET)
    mState = NOT_INITIALIZED;

  mLastProcessedState = mState;

  // Get Map Mutex -> Map cannot be changed
  unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

  if (mState==NOT_INITIALIZED) {
    if (mSensor==System::RGBD)
      StereoInitialization();
    else {
      if (usePattern)
        PatternInitialization();
      else
        MonocularInitialization();
    }

    if (mState!=OK)
      return;
  } else {
    // System is initialized. Track Frame.
    bool bOK;

    // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
    if (mState==OK) {
      // Local Mapping might have changed some MapPoints tracked in last frame
      CheckReplacedInLastFrame();

      if (!motion_model_->Started() || mCurrentFrame.mnId < mnLastRelocFrameId+2) {
        bOK = TrackReferenceKeyFrame();
      } else {
        bOK = TrackWithMotionModel();
        if (!bOK) {
          bOK = TrackReferenceKeyFrame();
          motion_model_->Restart();
        }
      }
    } else {
      bOK = Relocalization();
      motion_model_->Restart();
    }

    mCurrentFrame.mpReferenceKF = mpReferenceKF;

    // If we have an initial estimation of the camera pose and matching. Track the local map.
    if (bOK)
      bOK = TrackLocalMap();

    if (bOK)
      mState = OK;
    else
      mState = LOST;

    // If tracking were good, check if we insert a keyframe
    if (bOK) {

      // Update motion sensor
      if (!mLastFrame.GetPose().isZero())
        motion_model_->Update(mCurrentFrame.GetPose(), measurements_);
      else
        motion_model_->Restart();

      // Clean VO matches
      for (int i = 0; i<mCurrentFrame.N; i++) {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if (pMP)
          if (pMP->Observations()<1) {
            mCurrentFrame.mvbOutlier[i] = false;
            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
          }
      }

      // Delete temporal MapPoints
      for (list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++) {
        MapPoint* pMP = *lit;
        delete pMP;
      }
      mlpTemporalPoints.clear();

      // Check if we need to insert a new keyframe
      if (NeedNewKeyFrame())
        CreateNewKeyFrame();

      // We allow points with high innovation (considererd outliers by the Huber Function)
      // pass to the new keyframe, so that bundle adjustment will finally decide
      // if they are outliers or not. We don't want next frame to estimate its position
      // with those points so we discard them in the frame.
      for (int i = 0; i<mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
          mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
      }
    }

    // Reset if the camera get lost soon after initialization
    if (mState==LOST) {
      if (mpMap->KeyFramesInMap()<=5) {
        LOGD("Track lost soon after initialisation, reseting...");
        mpSystem->Reset();
        return;
      }
    }

    if (!mCurrentFrame.mpReferenceKF)
      mCurrentFrame.mpReferenceKF = mpReferenceKF;

    mLastFrame = Frame(mCurrentFrame);
  }

  // Store relative pose
  if (!mCurrentFrame.GetPose().isZero()) {
    Eigen::Matrix4d Tcr = mCurrentFrame.GetPose()*mCurrentFrame.mpReferenceKF->GetPoseInverse();
    lastRelativePose_ = Tcr;
  }
}



void Tracking::Track_IMU_Reinit() {
  cout << "[INFO] Tracking with IMU and Reinitilialization." << endl;
  used_imu_model = false;

  if (mState==NO_IMAGES_YET)
    mState = NOT_INITIALIZED;

  mLastProcessedState = mState;

  // Get Map Mutex -> Map cannot be changed
  unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

  if (mState==NOT_INITIALIZED) {
    MonocularInitializationIMU();
    if (mState!=OK)
      return;
    else{
      delete mpInitializer;
      mpInitializer = static_cast<Initializer*>(NULL);
      fill(mvIniMatches.begin(), mvIniMatches.end(),-1);
      mvIniP3D.clear();
    }

  } 
  else{
    new_imu_model.set_scale(mpReferenceKF->inertial_scale);  // ensure the scale after detecting a loop closure. TODO: Refine
    new_imu_model.predict(imu_measurements_, dt_); 
    curr_imu_prediction = new_imu_model.get_world_pose(mLastFrame.GetPoseInverse());
    std::cout << "[TEST/INFO] MM Estimation: " << curr_imu_prediction.block<3,1>(0,3).transpose() << endl;
    std::cout << "[INERTIAL POSE WORLD]:\n" << curr_imu_prediction << "\n" << std::endl;
    //if(mState!=NOT_INITIALIZED && mpMap->GetAllKeyFrames().size() > 20){ mState = OK_IMU;}
    
    // System is initialized. Track Frame.
    bool bOK;
    // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
    if (mState==OK) {
      // Local Mapping might have changed some MapPoints tracked in last frame
      CheckReplacedInLastFrame();

      if (!motion_model_->Started() || mCurrentFrame.mnId < mnLastRelocFrameId+2) {
        bOK = TrackReferenceKeyFrame();
      } else {
        bOK = TrackWithNewIMUModel();
        if (!bOK) {
          bOK = TrackReferenceKeyFrame();
          motion_model_->Restart();
          if (!bOK){
            last_valid_frame = Frame(mLastFrame);
            last_valid_wpose = last_valid_frame.GetPoseInverse();
            mCurrentFrame.SetPose(Converter::inverted_pose(curr_imu_prediction));
          }
        }
      }
      if (bOK){
        if (_scale_initializer.is_initialized()){
          new_imu_model.correct_pose(mCurrentFrame, mLastFrame, dt_);
        }else{
          new_imu_model.estimate_scale(mCurrentFrame, mLastFrame);
        }
      }
    } else if (mState==OK_IMU) {
      // Publicar posición (IMU int) mientras la parte visual trata de relocalizarse
      used_imu_model = true;
      
      bOK = false; //RelocalizationIMU();
      motion_model_->Restart();
      // new_imu_model.predict(imu_measurements_, dt_); 
      if (bOK){
        cout << "RELOCATE" << endl;
      }
      else{
        mCurrentFrame.SetPose(Converter::inverted_pose(curr_imu_prediction));
        MonocularReInitializationIMU(curr_imu_prediction);  
        bOK = (mState == OK);
        if (bOK){
          delete mpInitializer;
          mpInitializer = static_cast<Initializer*>(NULL);
          fill(mvIniMatches.begin(), mvIniMatches.end(),-1);
          mvIniP3D.clear();
        }else{

          if (_frames_last_fake_kf >= 5){
            _frames_last_fake_kf = 0;
            CreateNewKeyFrame(true);
          }else{
            _frames_last_fake_kf++;
          }
        }
      }
    }
    else {
      bOK = Relocalization();
      motion_model_->Restart();
    }

    mCurrentFrame.mpReferenceKF = mpReferenceKF;

    // If we have an initial estimation of the camera pose and matching. Track the local map.
    if (bOK){
      bOK = TrackLocalMap();
      if (!bOK){cout << "Error on track local map" << endl;}
    }
    if (bOK)
      mState = OK;
    else{
       mState = OK_IMU;
    }
    // If tracking were good, check if we insert a keyframe
    if (bOK) {

      // Update motion sensor
      if (!mLastFrame.GetPose().isZero())
        motion_model_->Update(mCurrentFrame.GetPose(), measurements_);
      else
        motion_model_->Restart();

      // Clean VO matches
      for (int i = 0; i<mCurrentFrame.N; i++) {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if (pMP)
          if (pMP->Observations()<1) {
            mCurrentFrame.mvbOutlier[i] = false;
            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
          }
      }

      // Delete temporal MapPoints
      for (list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++) {
        MapPoint* pMP = *lit;
        delete pMP;
      }
      mlpTemporalPoints.clear();

      // Check if we need to insert a new keyframe
      if (NeedNewKeyFrame()){
        // Estimate scale, rotation, and correct pose (?)
        CreateNewKeyFrame();
      }

      // We allow points with high innovation (considererd outliers by the Huber Function)
      // pass to the new keyframe, so that bundle adjustment will finally decide
      // if they are outliers or not. We don't want next frame to estimate its position
      // with those points so we discard them in the frame.
      for (int i = 0; i<mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
          mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
      }
    }

    // Reset if the camera get lost soon after initialization
    if (mState==LOST || mState==OK_IMU) {
      if (mpMap->KeyFramesInMap()<=5) {
        LOGD("Track lost soon after initialisation, reseting...");
        LOGD("TODO: Only remove new KFs");
        mpSystem->Reset();
        return;
      }
    }

    if (!mCurrentFrame.mpReferenceKF)
      mCurrentFrame.mpReferenceKF = mpReferenceKF;

    mLastFrame = Frame(mCurrentFrame);
  }

  // Store relative pose
  if (!mCurrentFrame.GetPose().isZero()) {
    Eigen::Matrix4d Tcr = mCurrentFrame.GetPose()*mCurrentFrame.mpReferenceKF->GetPoseInverse();
    lastRelativePose_ = Tcr;
  }
}

void Tracking::MonocularReInitializationIMU(const Matrix4d & curr_imu_prediction) {
  cout << "[[MONOCULAR INITIALIZER]]" << endl;
  if (!mpInitializer) {
    // Set Reference Frame
    if (mCurrentFrame.mvKeys.size()>100) {
      cout << "Creating initializer.." << endl;

      //mIniPoseInvIMU = new_imu_model.get_pose_world();
      mIniPoseInvIMU = curr_imu_prediction;

      mInitialFrame = Frame(mCurrentFrame);
      mLastFrame = Frame(mCurrentFrame);
      mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
      for (size_t i = 0; i<mCurrentFrame.mvKeysUn.size(); i++)
        mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

      if (mpInitializer)
        delete mpInitializer;

      mpInitializer =  new Initializer(mCurrentFrame, 1.0, 200);

      fill(mvIniMatches.begin(), mvIniMatches.end(),-1);
      return;
    }


  } else {
    // Try to initialize
    if ((int)mCurrentFrame.mvKeys.size()<=100) {
      delete mpInitializer;
      mpInitializer = static_cast<Initializer*>(NULL);
      fill(mvIniMatches.begin(), mvIniMatches.end(),-1);
      return;
    }

    // Find correspondences
    ORBmatcher matcher(0.9, true);
    int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);  
    
    cout << "\tTrying to relocalizate. Matches num: " << nmatches << endl;
    // Check if there are enough correspondences
    if (nmatches<100) {
      delete mpInitializer;
      mpInitializer = static_cast<Initializer*>(NULL);
      return;
    }
      
    Eigen::Matrix3d Rcw; // Current Camera Rotation
    Eigen::Vector3d tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

    bool initialize_process = mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated);
    if (!initialize_process){
      cout << "[INFO] Error estimando RT" <<endl;
    }
    if (initialize_process) {
      for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
        if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
          mvIniMatches[i]=-1;
          nmatches--;
        }
      }

      // 1- Create new (auxiliar) map
      Frame initial = Frame(mInitialFrame);
      Frame current = Frame(mCurrentFrame);

      initial.SetPose(Matrix4d::Identity());
      current.SetPose(Converter::toSE3(Rcw, tcw));

      _new_map = new Map();
      
      // Create KeyFrames
      KeyFrame* new_pKFini = new KeyFrame(initial, _new_map);
      KeyFrame* new_pKFcur = new KeyFrame(current, _new_map);
      // --
      // Insert KFs in the map
      _new_map->AddKeyFrame(new_pKFini);
      _new_map->AddKeyFrame(new_pKFcur);

      // Create MapPoints and asscoiate to keyframes
      for (size_t i = 0; i<mvIniMatches.size(); i++) {
        if (mvIniMatches[i] < 0)
          continue;

        //Create MapPoint.
        Eigen::Vector3d worldPos(Converter::toVector3d(mvIniP3D[i]));
        MapPoint* pMP = new MapPoint(worldPos, new_pKFcur, _new_map);

        new_pKFini->AddMapPoint(pMP, i);
        pMP->AddObservation(new_pKFini, i);
        new_pKFcur->AddMapPoint(pMP, mvIniMatches[i]);
        pMP->AddObservation(new_pKFcur, mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        current.mvpMapPoints[mvIniMatches[i]] = pMP;
        current.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        _new_map->AddMapPoint(pMP);
      }

      // Update Connections
      new_pKFini->UpdateConnections();
      new_pKFcur->UpdateConnections();


      // Bundle Adjustment
      LOGD("New Map created with %lu points", _new_map->MapPointsInMap());
      // Avoid this BA because it can modify the pose of the first frame, and we want it to remain at the origin.
      if (false){  
        cout << "[boubdleajustement]\n\tBEFORE:" << endl;
        cout << "\tInit (world): " << new_pKFini->GetPoseInverse().block<3,1>(0,3).transpose() << endl;
        cout << "\tCurr (world): " << new_pKFcur->GetPoseInverse().block<3,1>(0,3).transpose() << endl;
        Optimizer::GlobalBundleAdjustemnt(_new_map, 20); 
        cout << "\tAFTER:" << endl;
        cout << "\tInit (world): " << new_pKFini->GetPoseInverse().block<3,1>(0,3).transpose() << endl;
        cout << "\tCurr (world): " << new_pKFcur->GetPoseInverse().block<3,1>(0,3).transpose() << endl;
      
        std::cout << "[VISUAL INIT POSE WORLD]:\n" << new_pKFini->GetPoseInverse() << "\n" << std::endl;
        std::cout << "[VISUAL CURR POSE WORLD]:\n" << new_pKFcur->GetPoseInverse() << "\n" << std::endl;
      }
      // Move to origin again (if use GlobalBundleAdjustemnt)
      // Vector3d to_move = new_pKFini->GetPoseInverse().block<3,1>(0,3);
      // Matrix4d pose_init_after_ba = new_pKFini->GetPose();
      // Matrix4d pose_curr_after_ba = new_pKFcur->GetPose();

      // pose_init_after_ba.block<3,1>(0,3) = Vector3d(0,0,0);
      // pose_curr_after_ba.block<3,1>(0,3) = -new_pKFcur->GetRotation() * (new_pKFcur->GetPoseInverse().block<3,1>(0,3) - to_move);
      // new_pKFini->SetPose(pose_init_after_ba);
      // new_pKFcur->SetPose(pose_curr_after_ba);
      // cout << "\tAFTER MOVING TO ORIGIN AGAIN:" << endl;
      // cout << "\tInit (world): " << new_pKFini->GetPoseInverse().block<3,1>(0,3).transpose() << endl;
      // cout << "\tCurr (world): " << new_pKFcur->GetPoseInverse().block<3,1>(0,3).transpose() << endl;

      // ---


      // Set median depth to 1
      float medianDepth = new_pKFini->ComputeSceneMedianDepth(2);
      float invMedianDepth = 1.0f/medianDepth;
      cout << "Median depth: " << medianDepth << endl; 
      cout << "Tracked map points: " << new_pKFcur->TrackedMapPoints(1) << endl;
      if (medianDepth < 0 || new_pKFcur->TrackedMapPoints(1)<100) {
        LOGE("Wrong initialization, reseting new map...");
        // delete _new_map;
        return;
      }
      LOGD("New map initializate correctly");
      // return true;

      // Scale initial baseline
      Eigen::Matrix4d Tc2w_i = new_pKFini->GetPose();
      Tc2w_i.block<3, 1>(0, 3) = Tc2w_i.block<3, 1>(0, 3)*invMedianDepth;
      new_pKFini->SetPose(Tc2w_i);
      
      Eigen::Matrix4d Tc2w = new_pKFcur->GetPose();
      Tc2w.block<3, 1>(0, 3) = Tc2w.block<3, 1>(0, 3)*invMedianDepth; 
      new_pKFcur->SetPose(Tc2w);

      cout << "[POSES FROM IMU MODEL]" << endl;
      cout << "\tINIT POSE(world): " << mIniPoseInvIMU.block<3,1>(0,3).transpose() <<  endl;
      cout << "\tCURR POSE(world): " << curr_imu_prediction.block<3,1>(0,3).transpose() <<  endl;
      std::cout << "[INERTIAL INIT POSE WORLD]:\n" << mIniPoseInvIMU << "\n" << std::endl;
      std::cout << "[INERTIAL CURR POSE WORLD]:\n" << curr_imu_prediction << "\n" << std::endl;
        
      // NEW MAP WAS CREATED CORRECTLY
      // 2- Rotate and scales poses and map points to match with the inertial trajectory.
      Matrix3d R = mIniPoseInvIMU.block<3,3>(0,0); // rotation in world
      Vector3d T = mIniPoseInvIMU.block<3,1>(0,3); // traslation in world

      Matrix4d delta_cam =  new_pKFini->GetPoseInverse() * new_pKFcur->GetPose();

      new_pKFini->SetPose(Converter::inverted_pose(mIniPoseInvIMU));
      new_pKFcur->SetPose(delta_cam * new_pKFini->GetPose() );

      initial.SetPose(new_pKFini->GetPose());
      current.SetPose(new_pKFcur->GetPose());

      // 3- Check the consistency of the new map.
      //    If the angle formed by the inertial and the visual trajectory 
      //    (between the first and second keyframes of this new map) is greater
      //    than a certain angle, the map will be removed.
      Vector3d vector_vision = current.GetPoseInverse().block<3,1>(0,3) - initial.GetPoseInverse().block<3,1>(0,3);
      Vector3d vector_imu = curr_imu_prediction.block<3,1>(0,3) - mIniPoseInvIMU.block<3,1>(0,3);
      double diff_vector = abs(Estimator::angle(vector_imu, vector_vision, true));
      cout << "\n[Angle between IMU estimation vs Triangulation] " << diff_vector << endl;
      cout << "\t\t Vision 0: " << initial.GetPoseInverse().block<3,1>(0,3).transpose() << endl;
      cout << "\t\t Vision 1: " << current.GetPoseInverse().block<3,1>(0,3).transpose() << endl;
      cout << "\t\t IMU 0   : " << mIniPoseInvIMU.block<3,1>(0,3).transpose() << endl;
      cout << "\t\t IMU 1   : " << curr_imu_prediction.block<3,1>(0,3).transpose() << endl;

      if (diff_vector > _angle_th){
        printf("[FAIL] Angle between IMU and Vision traj is %.2fº, witch is greater than threshold (%.2fº).\n", diff_vector, _angle_th);
        return;
      }
    
      // Scale and rotate points
      vector<MapPoint*> vpAllMapPoints_ = new_pKFini->GetMapPointMatches();
      for (size_t iMP = 0; iMP<vpAllMapPoints_.size(); iMP++) {
        if (vpAllMapPoints_[iMP]) {
          MapPoint* pMP = vpAllMapPoints_[iMP];
          pMP->SetWorldPos(R*pMP->GetWorldPos()*invMedianDepth + T);
        }
      }

     
      if (!mpLocalMapper->SetNotStop(true))
        return;

      mInitialFrame.SetPose(new_pKFini->GetPose());
      mCurrentFrame.SetPose(new_pKFcur->GetPose());

      KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpMap);
      KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap);
      mpMap->AddKeyFrame(pKFini);
      mpMap->AddKeyFrame(pKFcur);

      for (size_t i = 0; i<mvIniMatches.size(); i++) {
        if (mvIniMatches[i] < 0)
          continue;

        //Create MapPoint.
        Eigen::Vector3d worldPos(Converter::toVector3d(mvIniP3D[i]));
        Vector3d new_pos_3d = R * (worldPos) * invMedianDepth + T; // R * (worldPos-to_move) * invMedianDepth + T;;
        MapPoint* pMP = new MapPoint(new_pos_3d, pKFcur, mpMap);

        pKFini->AddMapPoint(pMP, i);
        pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

        pMP->AddObservation(pKFini, i);
        pMP->AddObservation(pKFcur, mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
      }

      
      pKFini->UpdateConnections();
      pKFcur->UpdateConnections();

      mpFirstReloadKeyFrame = mpReferenceKF;
      if (_use_BA_on_reinit){
        cout << "[boubdleajustement mMap]\n\tBEFORE:" << endl;
        cout << "\tInit (world): " << pKFini->GetPoseInverse().block<3,1>(0,3).transpose() << endl;
        cout << "\tCurr (world): " << pKFcur->GetPoseInverse().block<3,1>(0,3).transpose() << endl;
        Optimizer::GlobalBundleAdjustemnt(mpMap, 20); 
        cout << "\tAFTER:" << endl;
        cout << "\tInit (world): " << pKFini->GetPoseInverse().block<3,1>(0,3).transpose() << endl;
        cout << "\tCurr (world): " << pKFcur->GetPoseInverse().block<3,1>(0,3).transpose() << endl;
      } 

      // 4- Scale the displacement between pKFini and pKFcur and map points
      //   to match the Vision-RealWorld ratio currently used (inertial_scale)
      Vector3d start_imu = mIniPoseInvIMU.block<3,1>(0,3);
      Vector3d end_imu =   curr_imu_prediction.block<3,1>(0,3);

      // Scalate curr position 
      Vector3d start_vis = pKFini->GetPoseInverse().block<3,1>(0,3);
      Vector3d end_vis = pKFcur->GetPoseInverse().block<3,1>(0,3);

      Vector3d delta_vision = end_vis - start_vis;
      Vector3d delta_imu = end_imu - start_imu;
      double lambda_a = new_imu_model.get_scale_factor();
      double lambda_b = Estimator::scale(delta_imu, delta_vision);
      cout << "[TEST] Lambda_B = " << lambda_b << endl;
      Vector3d new_end_vision = start_vis + delta_vision * (lambda_a / lambda_b);
      cout << "[TEST] Assert new scale (" << Estimator::scale(delta_imu, new_end_vision-start_vis) << ") is equal to old scale (" << lambda_a << ")" << endl;
      pKFcur->SetPose( Converter::inverted_pose(new_end_vision, pKFcur->GetPoseInverse().block<3,3>(0,0)) );

      pKFini->inertial_scale = new_imu_model.get_scale_factor();
      pKFcur->inertial_scale = new_imu_model.get_scale_factor();

      // Scalate points
      vector<MapPoint*> vpAllMapPoints = new_pKFini->GetMapPointMatches();
      for (size_t iMP = 0; iMP<vpAllMapPoints.size(); iMP++) {
        if (vpAllMapPoints[iMP]) {
          MapPoint* pMP = vpAllMapPoints[iMP];
          Vector3d MPw_B = pMP->GetWorldPos();
          Vector3d MPw_A = start_vis + (lambda_a / lambda_b) * (MPw_B - start_vis);
          pMP->SetWorldPos(MPw_A);
        }
      }


      // 5- Add the new map information to the general map.
      pKFini->ChangeParent(mpReferenceKF);
      pKFcur->ChangeParent(pKFini);
      mpFirstReloadKeyFrame = pKFini;
      cout << "PARENT: " << pKFini->GetParent() << endl;
      // X) Update local map
      // Add both KF to local map
      //mpLocalMapper->Release();
      mpLocalMapper->InsertKeyFrame(pKFini);
      mpLocalMapper->InsertKeyFrame(pKFcur);

      mInitialFrame.SetPose(pKFini->GetPose());
      mCurrentFrame.SetPose(pKFcur->GetPose());
      mnLastKeyFrameId = mCurrentFrame.mnId;
      mpLastKeyFrame = pKFcur;

      mvpLocalKeyFrames.push_back(pKFcur);
      mvpLocalKeyFrames.push_back(pKFini);
      mvpLocalMapPoints = mpMap->GetAllMapPoints();
      mpReferenceKF = pKFcur;
      mCurrentFrame.mpReferenceKF = pKFcur;

      mLastFrame = Frame(mCurrentFrame);

      mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
      // mpMap->mvpKeyFrameOrigins.push_back(pKFini);
      nKFs_on_reinit = mpMap->KeyFramesInMap();
      cout << "Map points tracks: " << mpReferenceKF->TrackedMapPoints(3) << endl;
      
      //UpdateLocalMap();
      mState = OK;
      return;
    }
    return;
  }
  return;
}


void Tracking::StereoInitialization() {
  if (mCurrentFrame.N>500) {
    // Set Frame pose to the origin
    mCurrentFrame.SetPose(Eigen::Matrix4d::Identity());

    // Create KeyFrame
    KeyFrame* pKFini = new KeyFrame(mCurrentFrame, mpMap);

    // Insert KeyFrame in the map
    mpMap->AddKeyFrame(pKFini);

    // Create MapPoints and asscoiate to KeyFrame
    for (int i = 0; i<mCurrentFrame.N; i++) {
      float z = mCurrentFrame.mvDepth[i];
      if (z > 0) {
        Eigen::Vector3d x3D = mCurrentFrame.UnprojectStereo(i);
        MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpMap);
        pNewMP->AddObservation(pKFini, i);
        pKFini->AddMapPoint(pNewMP, i);
        pNewMP->ComputeDistinctiveDescriptors();
        pNewMP->UpdateNormalAndDepth();
        mpMap->AddMapPoint(pNewMP);

        mCurrentFrame.mvpMapPoints[i]=pNewMP;
      }
    }

    LOGD("New map created with %lu points", mpMap->MapPointsInMap());

    mpLocalMapper->InsertKeyFrame(pKFini);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFini;

    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpMap->GetAllMapPoints();
    mpReferenceKF = pKFini;
    mCurrentFrame.mpReferenceKF = pKFini;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState = OK;
  }
}

void Tracking::MonocularInitialization() {
  if (!mpInitializer) {
    // Set Reference Frame
    if (mCurrentFrame.mvKeys.size()>100) {
      mInitialFrame = Frame(mCurrentFrame);
      mLastFrame = Frame(mCurrentFrame);
      mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
      for (size_t i = 0; i<mCurrentFrame.mvKeysUn.size(); i++)
        mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

      if (mpInitializer)
        delete mpInitializer;

      mpInitializer =  new Initializer(mCurrentFrame, 1.0, 200);

      fill(mvIniMatches.begin(), mvIniMatches.end(),-1);

      return;
    }
  } else {
    // Try to initialize
    if ((int)mCurrentFrame.mvKeys.size()<=100) {
      delete mpInitializer;
      mpInitializer = static_cast<Initializer*>(NULL);
      fill(mvIniMatches.begin(), mvIniMatches.end(),-1);
      return;
    }

    // Find correspondences
    ORBmatcher matcher(0.9, true);
    int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

    // Check if there are enough correspondences
    if (nmatches<100) {
      delete mpInitializer;
      mpInitializer = static_cast<Initializer*>(NULL);
      return;
    }

    Eigen::Matrix3d Rcw; // Current Camera Rotation
    Eigen::Vector3d tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

    if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated)) {
      for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
        if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
          mvIniMatches[i]=-1;
          nmatches--;
        }
      }

      // Set Frame Poses
      mInitialFrame.SetPose(Eigen::Matrix4d::Identity());
      Eigen::Matrix4d Tcw;
      Tcw.setIdentity();
      Tcw.block<3, 3>(0, 0) = Rcw;
      Tcw.block<3, 1>(0, 3) = tcw;
      mCurrentFrame.SetPose(Tcw);
      CreateInitialMapMonocular();
    }
  }
}

void Tracking::MonocularInitializationIMU() {
  new_imu_model.predict(imu_measurements_, dt_);
  if (!mpInitializer) {
    // Set Reference Frame
    if (mCurrentFrame.mvKeys.size()>100) {
      mIniPoseInvIMU = new_imu_model.get_pose_world();
      mInitialFrame = Frame(mCurrentFrame);
      mLastFrame = Frame(mCurrentFrame);
      mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
      for (size_t i = 0; i<mCurrentFrame.mvKeysUn.size(); i++)
        mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

      if (mpInitializer)
        delete mpInitializer;

      mpInitializer =  new Initializer(mCurrentFrame, 1.0, 200);

      fill(mvIniMatches.begin(), mvIniMatches.end(),-1);

      return;
    }
  } else {
    // Try to initialize
    if ((int)mCurrentFrame.mvKeys.size()<=100) {
      delete mpInitializer;
      mpInitializer = static_cast<Initializer*>(NULL);
      fill(mvIniMatches.begin(), mvIniMatches.end(),-1);
      return;
    }

    // Find correspondences
    ORBmatcher matcher(0.9, true);
    int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

    // Check if there are enough correspondences
    if (nmatches<100) {
      delete mpInitializer;
      mpInitializer = static_cast<Initializer*>(NULL);
      return;
    }

    Eigen::Matrix3d Rcw; // Current Camera Rotation
    Eigen::Vector3d tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

    if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated)) {
      for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
        if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
          mvIniMatches[i]=-1;
          nmatches--;
        }
      }

      // Set Frame Poses
      mInitialFrame.SetPose(Eigen::Matrix4d::Identity());
      Eigen::Matrix4d Tcw;
      Tcw.setIdentity();
      Tcw.block<3, 3>(0, 0) = Rcw;
      Tcw.block<3, 1>(0, 3) = tcw;
      mCurrentFrame.SetPose(Tcw);
      CreateInitialMapMonocular();

      if (mState == OK){
        Vector3d delta_imu = new_imu_model.get_pose_world().block<3,1>(0,3) - mIniPoseInvIMU.block<3,1>(0,3);
        Vector3d delta_v =   mpMap->GetKeyFrame(1)->GetPoseInverse().block<3,1>(0,3) - mpMap->GetKeyFrame(0)->GetPoseInverse().block<3,1>(0,3);

        double scale = Estimator::scale(delta_imu, delta_v);
        cout << "\n\t[TEST] Initial scale IMU-VISION: " << scale << endl;

        _scale_initializer.on_map_initialization(new_imu_model, scale, mInitialFrame, mCurrentFrame);
        if (_scale_initializer.is_initialized()){
          mpMap->GetKeyFrame(0)->inertial_scale = _scale_initializer.get_initial_scale();
          mpMap->GetKeyFrame(1)->inertial_scale = _scale_initializer.get_initial_scale();
        }
      }
    }
  }
}


void Tracking::CreateInitialMapMonocular() {
  // Create KeyFrames
  KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpMap);
  KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap);

  // Insert KFs in the map
  mpMap->AddKeyFrame(pKFini);
  mpMap->AddKeyFrame(pKFcur);

  // Create MapPoints and asscoiate to keyframes
  for (size_t i = 0; i<mvIniMatches.size(); i++) {
    if (mvIniMatches[i] < 0)
      continue;

    //Create MapPoint.
    Eigen::Vector3d worldPos(Converter::toVector3d(mvIniP3D[i]));

    MapPoint* pMP = new MapPoint(worldPos, pKFcur, mpMap);

    pKFini->AddMapPoint(pMP, i);
    pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

    pMP->AddObservation(pKFini, i);
    pMP->AddObservation(pKFcur, mvIniMatches[i]);

    pMP->ComputeDistinctiveDescriptors();
    pMP->UpdateNormalAndDepth();

    //Fill Current Frame structure
    mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
    mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

    //Add to Map
    mpMap->AddMapPoint(pMP);
  }

  // Update Connections
  pKFini->UpdateConnections();
  pKFcur->UpdateConnections();

  // Bundle Adjustment
  LOGD("New Map created with %lu points", mpMap->MapPointsInMap());

  cout << "[boubdleajustement]\n\tBEFORE:" << endl;
  cout << "\tInit (world): " << pKFini->GetPoseInverse().block<3,1>(0,3).transpose() << endl;
  cout << "\tCurr (world): " << pKFcur->GetPoseInverse().block<3,1>(0,3).transpose() << endl;
  Optimizer::GlobalBundleAdjustemnt(mpMap, 20); 
  cout << "\tAFTER:" << endl;
  cout << "\tInit (world): " << pKFini->GetPoseInverse().block<3,1>(0,3).transpose() << endl;
  cout << "\tCurr (world): " << pKFcur->GetPoseInverse().block<3,1>(0,3).transpose() << endl;

  // Set median depth to 1
  float medianDepth = pKFini->ComputeSceneMedianDepth(2);
  float invMedianDepth = 1.0f/medianDepth;
  cout << "medianDepth: " << medianDepth << endl;
  if (medianDepth < 0 || pKFcur->TrackedMapPoints(1)<100) {
    LOGE("Wrong initialization, reseting...");
    Reset();
    return;
  }

  // Scale initial baseline
  Eigen::Matrix4d Tc2w = pKFcur->GetPose();
  Tc2w.block<3, 1>(0, 3) = Tc2w.block<3, 1>(0, 3)*invMedianDepth;
  pKFcur->SetPose(Tc2w);

  // Scale points
  vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
  for (size_t iMP = 0; iMP<vpAllMapPoints.size(); iMP++) {
    if (vpAllMapPoints[iMP]) {
      MapPoint* pMP = vpAllMapPoints[iMP];
      pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
    }
  }

  mpLocalMapper->InsertKeyFrame(pKFini);
  mpLocalMapper->InsertKeyFrame(pKFcur);

  mCurrentFrame.SetPose(pKFcur->GetPose());
  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKFcur;

  mvpLocalKeyFrames.push_back(pKFcur);
  mvpLocalKeyFrames.push_back(pKFini);
  mvpLocalMapPoints = mpMap->GetAllMapPoints();
  mpReferenceKF = pKFcur;
  mCurrentFrame.mpReferenceKF = pKFcur;

  mLastFrame = Frame(mCurrentFrame);

  mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

  mpMap->mvpKeyFrameOrigins.push_back(pKFini);

  mState = OK;
}

void Tracking::PatternInitialization() {
  if (mCurrentFrame.N <= 500)
    return;

  // Search pattern
  if (mpPatternDetector.Detect(mCurrentFrame)) {

    // Set Frame pose from pattern
    Eigen::Matrix4d cam_pos = mpPatternDetector.GetRT();
    mCurrentFrame.SetPose(Eigen::Matrix4d::Identity());

    std::cout << "[INFO] Initial camera pose: [" << cam_pos(0, 3) << ", " << cam_pos(1, 3) << ", " << cam_pos(2, 3) << "]" << std::endl;

    // Create KeyFrame
    KeyFrame* pKFini = new KeyFrame(mCurrentFrame, mpMap);

    // Insert KeyFrame in the map
    mpMap->AddKeyFrame(pKFini);

    // Create MapPoints and asscoiate to keyframes
    vector<pair<int, Eigen::Vector3d>>& points = mpPatternDetector.GetPoints();
    for (auto p=points.begin(); p!=points.end(); p++) {
      int idx = p->first;

      // Calculate 3d point position from camera
      Eigen::Vector4d abspos(p->second(0), p->second(1), p->second(2), 1.0);
      Eigen::Vector4d relpos = cam_pos.inverse()*abspos;

      // Create MapPoint
      Eigen::Vector3d worldPos(relpos(0), relpos(1), relpos(2));
      MapPoint* pMP = new MapPoint(worldPos, pKFini, mpMap);
      pMP->AddObservation(pKFini, idx);
      pKFini->AddMapPoint(pMP, idx);
      pMP->ComputeDistinctiveDescriptors();
      pMP->UpdateNormalAndDepth();
      mpMap->AddMapPoint(pMP);

      // Fill Current Frame structure
      mCurrentFrame.mvpMapPoints[idx] = pMP;
    }

    LOGD("New map created from pattern with %lu points", mpMap->MapPointsInMap());

    mpLocalMapper->InsertKeyFrame(pKFini);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFini;

    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpMap->GetAllMapPoints();
    mpReferenceKF = pKFini;
    mCurrentFrame.mpReferenceKF = pKFini;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState = OK;
  }
}

void Tracking::CheckReplacedInLastFrame() {
  for (int i  = 0; i<mLastFrame.N; i++) {
    MapPoint* pMP = mLastFrame.mvpMapPoints[i];

    if (pMP) {
      MapPoint* pRep = pMP->GetReplaced();
      if (pRep) {
        mLastFrame.mvpMapPoints[i] = pRep;
      }
    }
  }
}


bool Tracking::TrackReferenceKeyFrame() {
  ORBmatcher matcher(0.7, true);

  // Set same pose
  Eigen::Matrix4d last_pose = mLastFrame.GetPose();
  mCurrentFrame.SetPose(last_pose);

  LOGD("Last pose: [%.4f, %.4f, %.4f]", last_pose(0, 3), last_pose(1, 3), last_pose(2, 3));

  // Align current and last image
  if (align_image_) {
    ImageAlign image_align;
    if (!image_align.ComputePose(mCurrentFrame, mpReferenceKF)) {
      LOGE("Image align failed");
      mCurrentFrame.SetPose(last_pose);
    }
  }

  // Project points seen in reference keyframe
  fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
  int nmatches = matcher.SearchByProjection(mCurrentFrame, mpReferenceKF, threshold_, mSensor!=System::RGBD);

  // If few matches, ignores alignment and uses a wider window search
  if (nmatches<20) {
    LOGD("Not enough matches [%d], double threshold", nmatches);
    mCurrentFrame.SetPose(last_pose);
    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
    nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2*threshold_, mSensor!=System::RGBD);
  }

  if (nmatches<20) {
    LOGD("Not enough matches [%d], tracking failed", nmatches);
    return false;
  }

  // Optimize frame pose with all matches
  Optimizer::PoseOptimization(&mCurrentFrame);

  // Discard outliers
  int nmatchesMap = 0;
  for (int i  = 0; i<mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        mCurrentFrame.mvbOutlier[i]=false;
        pMP->mbTrackInView = false;
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        nmatches--;
      } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
        nmatchesMap++;
    }
  }

  if (nmatchesMap<10) {
    LOGD("Not enough inliers [%d], tracking failed", nmatchesMap);
    return false;
  }

  return true;
}

void Tracking::UpdateLastFrame() {
  // Update pose according to reference keyframe
  KeyFrame* pRef = mLastFrame.mpReferenceKF;
  Eigen::Matrix4d Tlr = lastRelativePose_;

  mLastFrame.SetPose(Tlr*pRef->GetPose());
}

bool Tracking::TrackVisual(Eigen::Matrix4d predicted_pose) {
  mCurrentFrame.SetPose(predicted_pose);

  ORBmatcher matcher(0.9, true);

  // Align current and last image
  if (align_image_) {
    ImageAlign image_align;
    if (!image_align.ComputePose(mCurrentFrame, mLastFrame)) {
      LOGE("Image align failed");
      mCurrentFrame.SetPose(predicted_pose);
    }
  }

  // Project points seen in previous frame
  fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
  int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, threshold_, mSensor!=System::RGBD);
  first_proj = nmatches;

  // If few matches, ignores alignment and uses a wider window search
  if (nmatches<20) {
    LOGD("Not enough matches [%d], double threshold", nmatches);
    mCurrentFrame.SetPose(predicted_pose);
    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
    nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2*threshold_, mSensor!=System::RGBD);
    second_proj = nmatches;
  }

  if (nmatches<20) {
    LOGD("Not enough matches [%d], tracking failed", nmatches);
    return false;
  }

  // Optimize frame pose with all matches
  Optimizer::PoseOptimization(&mCurrentFrame);

  // Discard outliers
  int nmatchesMap = 0;
  for (int i  = 0; i<mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        mCurrentFrame.mvbOutlier[i]=false;
        pMP->mbTrackInView = false;
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        nmatches--;
      } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
        nmatchesMap++;
    }
  }

  inliers_on_pred = nmatchesMap;

  if (nmatchesMap<10) {
    LOGD("Not enough inliers [%d], tracking failed", nmatchesMap);
    return false;
  }

  return true;

}


int Tracking::TrackMMVisual(Frame &frame, bool & use_align_image) {
  Eigen::Matrix4d predicted_pose = frame.GetPose();

  ORBmatcher matcher(0.9, true);

  // Align current and last image
  if (use_align_image) {
    ImageAlign image_align;
    if (!image_align.ComputePose(frame, mLastFrame)) {
      LOGE("Image align failed");
      frame.SetPose(predicted_pose);
    }
  }

  // Project points seen in previous frame
  fill(frame.mvpMapPoints.begin(), frame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
  int nmatches = matcher.SearchByProjection(frame, mLastFrame, threshold_, mSensor!=System::RGBD);
  first_proj = nmatches;

  // If few matches, ignores alignment and uses a wider window search
  if (nmatches<20) {
    LOGD("Not enough matches [%d], double threshold", nmatches);
    frame.SetPose(predicted_pose);
    fill(frame.mvpMapPoints.begin(), frame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
    nmatches = matcher.SearchByProjection(frame, mLastFrame, 2*threshold_, mSensor!=System::RGBD);
    second_proj = nmatches;
  }

  if (nmatches<20) {
    LOGD("Not enough matches [%d], tracking failed", nmatches);
    return 0;
  }

  // Optimize frame pose with all matches
  Optimizer::PoseOptimization(&frame);

  // Discard outliers
  int nmatchesMap = 0;
  for (int i  = 0; i<frame.N; i++) {
    if (frame.mvpMapPoints[i]) {
      if (frame.mvbOutlier[i]) {
        MapPoint* pMP = frame.mvpMapPoints[i];

        frame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        frame.mvbOutlier[i]=false;
        pMP->mbTrackInView = false;
        pMP->mnLastFrameSeen = frame.mnId;
        nmatches--;
      } else if (frame.mvpMapPoints[i]->Observations() > 0)
        nmatchesMap++;
    }
  }

  if (nmatchesMap<10) {
    LOGD("Not enough inliers [%d], tracking failed", nmatchesMap);
    return 0;
  }

  return nmatchesMap;

}


void _test_synthetic_imu(IMU_Measurements syn_imu, double dt,
                         Frame &current_frame, Frame &last_frame){

}

void print_pos_att(Matrix4d pose, string text){
  Vector3d position = pose.block<3,1>(0,3);
  Quaterniond att(pose.block<3,3>(0,0));
  cout << text << endl;
  cout <<"\t pos: " << position.transpose() << endl;
  printf("\t att: [%.4f, %.4f, %.4f, %.4f]\n", att.x(), att.y(), att.z(), att.w());
}

bool Tracking::TrackWithNewIMUModel() {

  // Update last frame pose according to its reference keyframe
  UpdateLastFrame();

  // Predict pose with motion model
  Eigen::Matrix4d predicted_pose_cte_vel = motion_model_->Predict(mLastFrame.GetPose());
  LOGD("Predicted pose (model: cte velocity): [%.4f, %.4f, %.4f]", predicted_pose_cte_vel(0, 3), predicted_pose_cte_vel(1, 3), predicted_pose_cte_vel(2, 3));
  
  if (_use_hybrid_model && mpMap->KeyFramesInMap() >= _kf_to_use_hm){
    Eigen::Matrix4d predicted_pose_inertial = Converter::inverted_pose(curr_imu_prediction);
    LOGD("Predicted pose (model: inertial): [%.4f, %.4f, %.4f]", predicted_pose_inertial(0, 3), predicted_pose_inertial(1, 3), predicted_pose_inertial(2, 3));

    Frame frame_ctevel(mCurrentFrame);
    frame_ctevel.SetPose(predicted_pose_cte_vel);

    Frame frame_inertial(mCurrentFrame);
    frame_inertial.SetPose(predicted_pose_inertial);

    Timer time_cte_vel(true);
    int n_matches_visual = TrackMMVisual(frame_ctevel, align_image_);
    time_cte_vel.Stop();

    Timer time_inertial(true);
    int n_matches_inertial = TrackMMVisual(frame_inertial, _align_image_inertial);
    time_inertial.Stop();

    LOGD("Num. matches with cte. vel: %i. Process time: %.2fms", n_matches_visual, time_cte_vel.GetMsTime());
    LOGD("Num. matches with inertial: %i. Process time: %.2fms", n_matches_inertial, time_inertial.GetMsTime());
    _hyb_matches.clear(); _hyb_matches.push_back(n_matches_visual); _hyb_matches.push_back(n_matches_inertial);
    _hyb_times.clear(); _hyb_times.push_back(time_cte_vel.GetMsTime()); _hyb_times.push_back(time_inertial.GetMsTime());

    if (_full_hyb_mode){
      if (n_matches_inertial > 0){
        mCurrentFrame = frame_inertial;
        return true;
      }else if (n_matches_visual > 0) {
        mCurrentFrame = frame_ctevel;
        return true;
      }
      return false;

    }else{
      if (n_matches_inertial > 0 && n_matches_inertial >= n_matches_visual){
        mCurrentFrame = frame_inertial;
        return true;
      }
      if (n_matches_visual > 0 && n_matches_visual > n_matches_inertial){
        mCurrentFrame = frame_ctevel;
        return true;
      }
      cout << "\t[TEST] NO SE PUDO TRACKEAR CON NINGUN MODELO" << endl;
      return false;
    }
  }else{
    return TrackVisual(predicted_pose_cte_vel);
  }
}

bool Tracking::TrackWithMotionModel() {
  ORBmatcher matcher(0.9, true);

  // Update last frame pose according to its reference keyframe
  UpdateLastFrame();

  // Predict initial pose with motion model
  Eigen::Matrix4d predicted_pose = motion_model_->Predict(mLastFrame.GetPose());
  mCurrentFrame.SetPose(predicted_pose);

  LOGD("Predicted pose: [%.4f, %.4f, %.4f]", predicted_pose(0, 3), predicted_pose(1, 3), predicted_pose(2, 3));

  // Align current and last image
  if (align_image_) {
    ImageAlign image_align;
    if (!image_align.ComputePose(mCurrentFrame, mLastFrame)) {
      LOGE("Image align failed");
      mCurrentFrame.SetPose(predicted_pose);
    }
  }

  // Project points seen in previous frame
  fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
  int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, threshold_, mSensor!=System::RGBD);

  // If few matches, ignores alignment and uses a wider window search
  if (nmatches<20) {
    LOGD("Not enough matches [%d], double threshold", nmatches);
    mCurrentFrame.SetPose(predicted_pose);
    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
    nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2*threshold_, mSensor!=System::RGBD);
  }

  if (nmatches<20) {
    LOGD("Not enough matches [%d], tracking failed", nmatches);
    return false;
  }

  // Optimize frame pose with all matches
  Optimizer::PoseOptimization(&mCurrentFrame);

  // Discard outliers
  int nmatchesMap = 0;
  for (int i  = 0; i<mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        mCurrentFrame.mvbOutlier[i]=false;
        pMP->mbTrackInView = false;
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        nmatches--;
      } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
        nmatchesMap++;
    }
  }

  if (nmatchesMap<10) {
    LOGD("Not enough inliers [%d], tracking failed", nmatchesMap);
    return false;
  }

  return true;
}

bool Tracking::TrackLocalMap() {
  // We have an estimation of the camera pose and some map points tracked in the frame.
  // We retrieve the local map and try to find matches to points in the local map.

  UpdateLocalMap();

  SearchLocalPoints();

  // Optimize Pose
  Optimizer::PoseOptimization(&mCurrentFrame);
  mnMatchesInliers = 0;

  // Update MapPoints Statistics
  for (int i = 0; i<mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (!mCurrentFrame.mvbOutlier[i]) {
        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
          mnMatchesInliers++;
      }
    }
  }

  inliers_on_localmap = mnMatchesInliers;
  // Decide if the tracking was succesful
  if (mnMatchesInliers<15) {
    LOGD("Not enough points tracked [%d], tracking failed", mnMatchesInliers);
    return false;
  }

  return true;
}


bool Tracking::NeedNewKeyFrame() {
  // If Local Mapping is freezed by a Loop Closure do not insert keyframes
  if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    return false;

  const int nKFs = mpMap->KeyFramesInMap();

  // Do not insert keyframes if not enough frames have passed from last relocalisation
  if (mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
    return false;

  // Tracked MapPoints in the reference keyframe
  int nMinObs = 3;
  if (nKFs<=2 || nKFs == nKFs_on_reinit)
    nMinObs=2;
  if (nKFs==1 && usePattern)
    nMinObs=1;
  int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);
  // Local Mapping accept keyframes?
  bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

  // Check how many "close" points are being tracked and how many could be potentially created.
  int nNonTrackedClose = 0;
  int nTrackedClose= 0;
  if (mSensor==System::RGBD) {
    for (int i  = 0; i<mCurrentFrame.N; i++) {
      if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i]<mThDepth) {
        if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
          nTrackedClose++;
        else
          nNonTrackedClose++;
      }
    }
  }

  bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

  // Thresholds
  float thRefRatio = 0.75f;
  if (nKFs<2)
    thRefRatio = 0.4f;

  if (mSensor!=System::RGBD)
    thRefRatio = 0.9f;

  // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
  const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId+mMaxFrames;
  // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
  const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
  //Condition 1c: tracking is weak
  const bool c1c =  mSensor==System::RGBD && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
  // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
  const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

  if ((c1a||c1b||c1c)&&c2) {
    // If the mapping accepts keyframes, insert keyframe.
    // Otherwise send a signal to interrupt BA
    if (bLocalMappingIdle) {
      return true;
    } else {
      mpLocalMapper->InterruptBA();

      if (mSensor==System::RGBD) {
        if (mpLocalMapper->KeyframesInQueue()<3)
          return true;
        else
          return false;
      } else
        return false;
    }
  } else
    return false;
}

void Tracking::CreateNewKeyFrame(bool is_fake) {
  if (!mpLocalMapper->SetNotStop(true))
    return;

  KeyFrame* pKF = new KeyFrame(mCurrentFrame, mpMap);
  if (is_fake){
    pKF->set_fake(true);
    pKF->ChangeParent(mpReferenceKF);
    pKF->inertial_scale = mpReferenceKF->inertial_scale;
  }
  
  if (mSensor==System::MONOCULAR_IMU_NEW && !is_fake){
    double mean_scale = new_imu_model.scale_buffer_mean();
    new_imu_model.scale_buffer_clear();
    cout << "\n\t[TEST] SCALE MEAN ON NEW KF (" << mpMap->GetAllKeyFrames().size()+1<< "): " << mean_scale << endl << endl;

    //**
    // if ( mpMap->GetAllKeyFrames().size() == 10){
    //   cout << "[TEST] Initialize motion model and reset scale" << endl;
    //   double dt = pKF->mTimestamp - mpReferenceKF->mTimestamp;
    //   new_imu_model.initialize(pKF->GetPoseInverse(), mpReferenceKF->GetPoseInverse(), dt);
    //   _scale_initializer.clear_buffer();
    // }

    // if (mpMap->GetAllKeyFrames().size() > 10 and not _scale_initializer.is_initialized()){
    //   _scale_initializer.update(new_imu_model, mean_scale);
    //   if (_scale_initializer.is_initialized()){
    //     for (KeyFrame* pKFi : mpMap->GetAllKeyFrames()){
    //       pKFi->inertial_scale = _scale_initializer.get_initial_scale();
    //     }
    //     pKF->inertial_scale = _scale_initializer.get_initial_scale();
    //   }
    // }
    // else if (_scale_initializer.is_initialized()){
    //   double curr_scale = mpReferenceKF->inertial_scale;
    //   cout << "\t[TEST] CURR SCALE: " << curr_scale << endl;

    //   if (update_scale){
    //     curr_scale = curr_scale * (1-_scale_update_factor) + mean_scale * _scale_update_factor;
    //   }
    //   cout << "\t[TEST] NEW SCALE: " << curr_scale << endl;
    //   new_imu_model.set_scale(curr_scale);
    //   pKF->inertial_scale = curr_scale;
    // }

    if (not _scale_initializer.is_initialized()){
      _scale_initializer.update(new_imu_model, mean_scale);
      if (_scale_initializer.is_initialized()){
        // Check relation KF-Frames (time)
        for (KeyFrame* pKFi : mpMap->GetAllKeyFrames()){
          pKFi->inertial_scale = _scale_initializer.get_initial_scale();
        }
        pKF->inertial_scale = _scale_initializer.get_initial_scale();
      }
    }
    else{
      double curr_scale = mpReferenceKF->inertial_scale;
      cout << "\t[TEST] CURR SCALE: " << curr_scale << endl;

      if (update_scale){
        curr_scale = curr_scale * (1-_scale_update_factor) + mean_scale * _scale_update_factor;
      }
      cout << "\t[TEST] NEW SCALE: " << curr_scale << endl;
      new_imu_model.set_scale(curr_scale);
      pKF->inertial_scale = curr_scale;
    }
  }

  mpReferenceKF = pKF;
  mCurrentFrame.mpReferenceKF = pKF;


  if (mSensor==System::RGBD) {
    mCurrentFrame.UpdatePoseMatrices();

    // We sort points by the measured depth by the stereo/RGBD sensor.
    // We create all those MapPoints whose depth < mThDepth.
    // If there are less than 100 close points we create the 100 closest.
    vector<pair<float, int> > vDepthIdx;
    vDepthIdx.reserve(mCurrentFrame.N);
    for (int i = 0; i<mCurrentFrame.N; i++) {
      float z = mCurrentFrame.mvDepth[i];
      if (z > 0) {
        vDepthIdx.push_back(make_pair(z, i));
      }
    }

    if (!vDepthIdx.empty()) {
      sort(vDepthIdx.begin(), vDepthIdx.end());

      int nPoints = 0;
      for (size_t j = 0; j < vDepthIdx.size(); j++) {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if (!pMP)
          bCreateNew = true;
        else if (pMP->Observations()<1) {
          bCreateNew = true;
          mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }

        if (bCreateNew) {
          Eigen::Vector3d x3D = mCurrentFrame.UnprojectStereo(i);
          MapPoint* pNewMP = new MapPoint(x3D, pKF, mpMap);
          pNewMP->AddObservation(pKF, i);
          pKF->AddMapPoint(pNewMP, i);
          pNewMP->ComputeDistinctiveDescriptors();
          pNewMP->UpdateNormalAndDepth();
          mpMap->AddMapPoint(pNewMP);

          mCurrentFrame.mvpMapPoints[i]=pNewMP;
          nPoints++;
        } else {
          nPoints++;
        }

        if (vDepthIdx[j].first>mThDepth && nPoints>100)
          break;
      }
    }
  }

  mpLocalMapper->InsertKeyFrame(pKF);

  mpLocalMapper->SetNotStop(false);

  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints() {
  // Do not search map points already matched
  for (vector<MapPoint*>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++) {
    MapPoint* pMP = *vit;
    if (pMP) {
      if (pMP->isBad()) {
        *vit = static_cast<MapPoint*>(NULL);
      } else {
        pMP->IncreaseVisible();
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        pMP->mbTrackInView = false;
      }
    }
  }

  int nToMatch = 0;

  // Project points in frame and check its visibility
  for (vector<MapPoint*>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit!=vend; vit++) {
    MapPoint* pMP = *vit;
    if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
      continue;
    if (pMP->isBad())
      continue;
    // Project (this fills MapPoint variables for matching)
    if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
      pMP->IncreaseVisible();
      nToMatch++;
    }
  }

  if (nToMatch > 0) {
    ORBmatcher matcher(0.8);
    int th = 1;
    if (mSensor==System::RGBD)
      th=3;
    // If the camera has been relocalised recently, perform a coarser search
    if (mCurrentFrame.mnId<mnLastRelocFrameId+2)
      th=5;
    matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
  }
}

void Tracking::UpdateLocalMap() {
  // This is for visualization
  mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

  // Update
  UpdateLocalKeyFrames();
  UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints() {
  mvpLocalMapPoints.clear();

  for (vector<KeyFrame*>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++) {
    KeyFrame* pKF = *itKF;
    const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

    for (vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++) {
      MapPoint* pMP = *itMP;
      if (!pMP)
        continue;
      if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
        continue;
      if (!pMP->isBad()) {
        mvpLocalMapPoints.push_back(pMP);
        pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
      }
    }
  }
}


void Tracking::UpdateLocalKeyFrames() {

  // Each map point vote for the keyframes in which it has been observed
  map<KeyFrame*, int> keyframeCounter;
  for (int i = 0; i<mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
      if (!pMP->isBad()) {
        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
        for (map<KeyFrame*, size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
          keyframeCounter[it->first]++;
      } else {
        mCurrentFrame.mvpMapPoints[i]=NULL;
      }
    }
  }

  if (keyframeCounter.empty())
    return;

  int max = 0;
  KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

  mvpLocalKeyFrames.clear();
  mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

  // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
  for (map<KeyFrame*, int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++) {
    KeyFrame* pKF = it->first;

    if (pKF->isBad())
      continue;

    if (it->second>max) {
      max=it->second;
      pKFmax=pKF;
    }

    mvpLocalKeyFrames.push_back(it->first);
    pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
  }

  // Include also some not-already-included keyframes that are neighbors to already-included keyframes
  for (vector<KeyFrame*>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++) {
    // Limit the number of keyframes
    if (mvpLocalKeyFrames.size()>80)
      break;

    KeyFrame* pKF = *itKF;

    const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

    for (vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++) {
      KeyFrame* pNeighKF = *itNeighKF;
      if (!pNeighKF->isBad()) {
        if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pNeighKF);
          pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    const set<KeyFrame*> spChilds = pKF->GetChilds();
    for (set<KeyFrame*>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++) {
      KeyFrame* pChildKF = *sit;
      if (!pChildKF->isBad()) {
        if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pChildKF);
          pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    KeyFrame* pParent = pKF->GetParent();
    if (pParent) {
      if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(pParent);
        pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        break;
      }
    }

  }

  if (pKFmax) {
    mpReferenceKF = pKFmax;
    mCurrentFrame.mpReferenceKF = mpReferenceKF;
  }
}

bool Tracking::RelocalizationIMU(){
  ORBmatcher matcher(0.75, true);
  int nmatches, nGood;

  
  // Compare to all keyframes starting from the last one
  vector<KeyFrame*> kfs = mpMap->GetAllKeyFrames();
  for (auto it=kfs.rbegin(); it != kfs.rend(); it++) {
    KeyFrame* kf = *it;


    //mCurrentFrame.SetPose(new_imu_model.get_pose_cam());
    // Try to align current frame and candidate keyframe
    ImageAlign image_align;
    if (!image_align.ComputePose(mCurrentFrame, kf, true))
      continue;

    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    nmatches = matcher.SearchByProjection(mCurrentFrame, kf, threshold_, mSensor!=System::RGBD);
    if (nmatches < 80)
      continue;

    // Optimize frame pose with all matches
    nGood = Optimizer::PoseOptimization(&mCurrentFrame);
    if (nGood < 40)
      continue;

    mnLastRelocFrameId = mCurrentFrame.mnId;
    return true;
  }

  mCurrentFrame.SetPose(new_imu_model.get_pose_cam());
  return false;
}

bool Tracking::Relocalization() {
  ORBmatcher matcher(0.75, true);
  int nmatches, nGood;

  // Compare to all keyframes starting from the last one
  vector<KeyFrame*> kfs = mpMap->GetAllKeyFrames();
  for (auto it=kfs.rbegin(); it != kfs.rend(); it++) {
    KeyFrame* kf = *it;

    mCurrentFrame.SetPose(kf->GetPose());
    //mCurrentFrame.SetPose(new_imu_model.get_pose_cam());
    // Try to align current frame and candidate keyframe
    ImageAlign image_align;
    if (!image_align.ComputePose(mCurrentFrame, kf, true))
      continue;

    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    nmatches = matcher.SearchByProjection(mCurrentFrame, kf, threshold_, mSensor!=System::RGBD);
    if (nmatches < 80)
      continue;

    // Optimize frame pose with all matches
    nGood = Optimizer::PoseOptimization(&mCurrentFrame);
    if (nGood < 40)
      continue;

    mnLastRelocFrameId = mCurrentFrame.mnId;
    return true;
  }

  return false;
}

void Tracking::Reset() {

  LOGD("System Reseting");

  // Reset Local Mapping
  LOGD("Reseting Local Mapper...");
  mpLocalMapper->RequestReset();

  // Reset Loop Closing
  if (mpLoopClosing) {
    LOGD("Reseting Loop Closing...");
    mpLoopClosing->RequestReset();
  }

  // Clear Map (this erase MapPoints and KeyFrames)
  mpMap->clear();

  KeyFrame::nNextId = 0;
  Frame::nNextId = 0;
  mState = NO_IMAGES_YET;

  if (mpInitializer) {
    delete mpInitializer;
    mpInitializer = static_cast<Initializer*>(NULL);
  }

  lastRelativePose_.setZero();
  motion_model_->Restart();
}

void Tracking::InformOnlyTracking(const bool &flag) {
  mbOnlyTracking = flag;
}

void Tracking::PatternCellSize(double w, double h){
	mpPatternDetector.SetCellSizeW(w);
	mpPatternDetector.SetCellSizeH(h);
}

}  // namespace SD_SLAM
