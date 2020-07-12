
#ifndef SD_SLAM_POSITIONESTIMATOR_H_
#define SD_SLAM_POSITIONESTIMATOR_H_

#include "inertial/IMU_Measurements.h"
#include "inertial/tools/filters.h"
#include <Eigen/Dense>

using namespace Eigen;

class PositionEstimator{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  PositionEstimator(const bool & use_linear_acc = false);

  PositionEstimator(const double & alpha_acceleration, 
                    const Eigen::Vector3d & gravity_vector = Eigen::Vector3d(0, 0, 9.80655));

  inline Eigen::Vector3d position(){return Vector3d(_position);}
  inline Eigen::Vector3d velocity(){return Vector3d(_velocity);}
  inline Eigen::Vector3d acceleration(){return Vector3d(_acceleration);}
  inline void set_gravity(const Vector3d & g){_gravity = g;}
  
  Eigen::Vector3d update(const Eigen::Vector3d & linear_acceleration, 
                         const double & dt,
                         const bool & verbose=false);

  Eigen::Vector3d update(const Eigen::Vector3d & acceleration, 
                         const Eigen::Quaterniond & sensor_attitude, 
                         const double & dt,
                         const bool & verbose=false);

  void correct_pos_and_vel(const Vector3d & curr_pos, const Vector3d & last_pose, double dt);
  

  /**
    Return the acceleration without the gravity force, in other words, return the linear acceleration.

    @param acceleration from the accelerometer (NWU coordinate system).
    @param sensor_attitude is the current attitude.
    @param verbose is optional argument to display information of the process.

    @return linear acceleration
  */
  Eigen::Vector3d remove_gravity(const Eigen::Vector3d & acceleration, 
                                 const Eigen::Quaterniond & sensor_attitude,
                                 const bool & verbose=false);

 private:
  LowPassFilter _lpf_acceleration;
  Eigen::Vector3d _gravity;
  
  Eigen::Vector3d _last_acceleration;
  Eigen::Vector3d _acceleration;
  Eigen::Vector3d _velocity;
  Eigen::Vector3d _position;

  bool _use_linear_acc;
  bool _initialized;
  int _it;
  Eigen::Vector3d _last_velocity;
};

#endif  // SD_SLAM_POSITIONESTIMATOR_H_
