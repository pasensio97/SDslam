
#ifndef SD_SLAM_PREDICTIONMODELS_H_
#define SD_SLAM_PREDICTIONMODELS_H_

#include <Eigen/Dense>
#include "inertial/attitude_estimators/Madgwick.h"
#include "inertial/PositionEstimator.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "inertial/tools/filters.h"
#include <iostream>
#include <vector>

using namespace Eigen;
using namespace SD_SLAM;


class GT_PredictionModel{
 private:
  Matrix3d rotation_gps_to_slam;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void estimate_rotation_gps_to_slam(Frame & first_frame, Frame & curr_frame);
  Matrix4d predict(Frame & curr_frame, Frame & last_frame);
};


class SyntheticSensor_PredictionModel{
 private:
  PositionEstimator pos_estimator;
  Madgwick att_estimator;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

    // imu data on CAM...
    pos_estimator.update(imu_data.acceleration(), dt);
    pose_predicted.block<3,1>(0,3) = pos_estimator.position();

    return pose_predicted;
  }
};

class IMU_model{

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PositionEstimator _pos_estimator;
  Madgwick _att_estimator;
  Matrix3d _R_imu2w;
  LowPassFilter acc_lpf;
  LowPassFilter ratio_lpf;
  int it;
  double _ratio;
  Vector3d _last_vel, _linear_acc;
  bool _remove_gravity;

// public:
  Vector3d position;
  Vector3d position_cam;
  Matrix3d attitude;

  IMU_model(const double & mad_gain);

  inline void set_remove_gravity(bool flag){_remove_gravity = flag;}
  inline void set_rotation_imu_to_slamworld(const Matrix3d & rot){
    _R_imu2w = Matrix3d(rot);
  }
  inline void set_ratio(double ratio){
    _ratio = ratio;
  }
  Matrix4d predict(IMU_Measurements & imu, double & dt, Frame & curr_frame, Frame & last_frame);
  Matrix4d predict(IMU_Measurements & imu, double & dt);
  void correct_pose(Frame & curr_frame, Frame & last_frame, double dt);

};


class new_IMU_model{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  LowPassFilter _acc_lpf;
  bool _acc_due_grav;
  Madgwick _att_estimator;
  Vector3d _gravity;
  Matrix3d _R_imu_to_world;

  double _scale;
  Vector3d _position;
  Vector3d _velocity;

  Vector3d _last_position;
  Vector3d _last_velocity;
  Vector3d _last_acceleration;
  bool _acc_init;  // Only for test with _last_acceleration

  Vector3d _delta_pos;
  std::vector<double> _scale_buffer;
  

  Vector3d _remove_gravity(const Vector3d & acc, const Quaterniond & attitude);
  Vector3d _remove_gravity_test(const Vector3d & acc, const Quaterniond & attitude);
  Vector3d _remove_gravity_test_2(const Vector3d & acc, const Quaterniond & attitude);
  void _update_poses();
  
 public:
  Matrix4d pose_cam, pose_world, pose_imu;

  new_IMU_model(const double & acc_lpf_gain, const bool &  remove_gravity, const double & mad_gain);
  inline void set_rotation_imu_to_world(const Matrix3d &R){_R_imu_to_world = R;};
  inline void set_gravity(Vector3d & g){_gravity = g;};
  inline void set_scale(double & scale){_scale = scale;};
  inline void set_remove_gravity_flag(const bool flag){_acc_due_grav = flag;};
  inline void set_att_gain(double gain){_att_estimator.set_gain(gain);};
  inline void set_attitude(const Quaterniond & att){_att_estimator.set_orientation(att);};

  inline Matrix3d get_rotation_imu_to_world(){return Matrix3d(_R_imu_to_world);};
  inline bool is_gravity_removed(){return _acc_due_grav;};
  inline double get_scale_factor(){return _scale;};
  inline double att_gain(){return _att_estimator.gain();};
  inline Madgwick get_att_estimator(){return _att_estimator;};

  inline Matrix4d get_pose_cam(){return Matrix4d(pose_cam);};
  inline Matrix4d get_pose_world(){return Matrix4d(pose_world);};
  inline Matrix4d get_pose_imu(){return Matrix4d(pose_imu);};
  // Test

  inline void set_velocity(const Vector3d & vel){_velocity = Vector3d(vel)*_scale;}
  inline Vector3d get_position_imu(){return Vector3d(_position);}
  inline Vector3d get_position_slam(){return _R_imu_to_world * _position;}
  inline Vector3d get_last_position_imu(){return Vector3d(_last_position);}
  inline Vector3d get_last_position_slam(){return _R_imu_to_world * _last_position;}

  double estimate_scale(const Frame & curr_frame, const Frame & last_frame, bool add_to_buffer=true);
  double estimate_scale(const Vector3d delta_pos_slam, bool add_to_buffer=true);
  inline void add_scale_to_buffer(double scale){_scale_buffer.push_back(scale);}
  double scale_buffer_mean();
  void scale_buffer_clear(bool full=false);
  /**
   * imu must be stay on NWU coordinate system
  */
  Matrix4d predict(IMU_Measurements & imu, double & dt);
  void correct_pose(const Frame & curr_frame, const Frame & last_frame, double dt, bool estimate_and_save_scale=true);
  void reset();
  void initialize(const Matrix4d & curr_world_pose, const Matrix4d & last_world_pose, double dt);

  Matrix4d get_world_pose(const Matrix4d & last_world_pose);
};


class GPS_IMU_model{
  
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Madgwick _att_estimator;
  Matrix4d _pose, _wpose;

  GPS_IMU_model(const double & mad_gain);
  Matrix4d estimate_pose(IMU_Measurements & imu, double & dt, Frame & curr_frame, Frame & last_frame, double scale);
  inline Matrix4d get_pose(){return _pose;};
  inline Matrix4d get_world_pose(){return _wpose;};
};

#endif