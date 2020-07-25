
#ifndef SD_SLAM_ESTIMATOR_H_
#define SD_SLAM_ESTIMATOR_H_

#include <iostream>
#include <vector>
#include <Eigen/Dense>
using namespace Eigen;


class Estimator{
 public:
  /**
   * Estimates the scale relationship between two trajctories A and B.
   * 
   * @param start_A position of traj A at time k-1
   * @param end_A position of traj A at time k
   * @param start_B position of traj B at time k-1
   * @param end_B position of traj B at time k
   * @return factor to scalate A to B
  */
  static double scale(const Vector3d & start_A, const Vector3d & end_A, const Vector3d & start_B, const Vector3d & end_B);

  /**
   * Estimates the scale relationship between two trajctories A and B.
   * 
   * @param delta_A vector of traj A between time k and k-1
   * @param delta_B vector of traj B between time k and k-1
   * @return factor to scalate A to B 
  */
  static double scale(const Vector3d & delta_A, const Vector3d & delta_B);

  /**
   * Estimates the angle between 2 vectors.
   * 
   * @param vector_A 
   * @param vector_B 
   * @param in_degrees return the angle in degrees
   * @return angle between A and B
  */
  static double angle(const Vector3d & vector_A, const Vector3d & vector_B, bool in_degrees=false);


  /**
   * Estimates the rotation relationship between two trajctories A and B.
   * 
   * @param start_A position of traj A at time k-1
   * @param end_A position of traj A at time k
   * @param start_B position of traj B at time k-1
   * @param end_B position of traj B at time k
   * @return matrix to rotated A to B
  */
  static Matrix3d rotation(const Vector3d & start_A, const Vector3d & end_A, const Vector3d & start_B, const Vector3d & end_B);


  static Vector3d correction_scale_factor(
      const Vector3d & start_imu, const Vector3d & end_imu,
      const Vector3d & start_vision, const Vector3d & end_vision, 
      const double & expected_scale);

  static double mean(std::vector<double> vec);
  static Quaterniond quat_difference(const Quaterniond & q_start, const Quaterniond & q_end);
};


#endif  // SD_SLAM_ESTIMATOR_H_