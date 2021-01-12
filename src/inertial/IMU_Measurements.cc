
#include "inertial/IMU_Measurements.h"


IMU_Measurements::IMU_Measurements(){
  _timestamp = 0.0;
  _acceleration = Vector3d(0,0,0);
  _angular_velocity = Vector3d(0,0,0);
}

IMU_Measurements::IMU_Measurements(const double &timestamp, 
                                   const Eigen::Vector3d &acceleration, 
                                   const Eigen::Vector3d &angular_velocity)
{
  _timestamp = timestamp;
  _acceleration = Vector3d(acceleration);
  _angular_velocity = Vector3d(angular_velocity);
}

IMU_Measurements IMU_Measurements::apply_R(const Eigen::Matrix3d& R){
  Vector3d acc = R * _acceleration;
  Vector3d gyr = R * _angular_velocity;
  return IMU_Measurements(_timestamp, acc, gyr);
}

