
#ifndef SD_SLAM_PREDICTIONMODELS_H_
#define SD_SLAM_PREDICTIONMODELS_H_

#include <Eigen/Dense>
#include "inertial/Madgwick.h"
#include "inertial/PositionEstimator.h"
#include "Frame.h"
#include "inertial/tools/filters.h"

using namespace Eigen;
using namespace SD_SLAM;


class GT_PredictionModel{
 private:
  Matrix3d rotation_gps_to_slam;

 public:
  void estimate_rotation_gps_to_slam(Frame & first_frame, Frame & curr_frame);
  Matrix4d predict(Frame & curr_frame, Frame & last_frame);
};


class SyntheticSensor_PredictionModel{
 private:
  PositionEstimator pos_estimator;
  Madgwick att_estimator;

 public:
  SyntheticSensor_PredictionModel(double madgwick_gain):
    pos_estimator(true),
    att_estimator(madgwick_gain) 
  {}

  Matrix4d predict(IMU_Measurements &imu_data, double &dt, Frame last_frame){
    Matrix4d pose_predicted = Matrix4d::Identity();

    // imu data to NWU ....
    att_estimator.set_orientation_from_frame(last_frame.GetPose());
    
    Vector3d acc = imu_data.acceleration();
    Vector3d gyr = imu_data.angular_velocity();
    att_estimator.update(acc, gyr, dt);
    pose_predicted.block<3,3>(0,0) = att_estimator.get_local_orientation().toRotationMatrix();

    // imu data on CAM... JOOOOODER!
    pos_estimator.update(imu_data.acceleration(), dt);
    pose_predicted.block<3,1>(0,3) = pos_estimator.position();

    return pose_predicted;
  }
};

class IMU_model{

 public:
  PositionEstimator _pos_estimator;
  Madgwick _att_estimator;
  Matrix3d _R_imu2w;
  LowPassFilter acc_lpf;
  LowPassFilter ratio_lpf;
  int it;
  double _ratio;
  Vector3d _last_vel, _linear_acc;

// public:
  Vector3d position;
  Vector3d position_cam;
  Matrix3d attitude;

  IMU_model(const double & mad_gain);

  inline void set_rotation_imu_to_slamworld(Matrix3d rot){
    _R_imu2w = rot;
  }
  inline void set_ratio(double ratio){
    _ratio = ratio;
  }
  Matrix4d predict(IMU_Measurements & imu, double & dt, Frame & curr_frame, Frame & last_frame);
  void correct_pose(Frame & curr_frame, Frame & last_frame, double dt);

};

#endif