
#ifndef SD_SLAM_MADGWICK_H_
#define SD_SLAM_MADGWICK_H_

#include <math.h>
#include <Eigen/Dense>

using namespace Eigen;

class Madgwick{
  // based on http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Madgwick(double gain);

  void set_orientation(const Quaterniond & orientation);
  void set_orientation_from_frame(const Matrix4d & local_pose);
  const Quaterniond get_orientation();
  Quaterniond get_world_orientation();
  Quaterniond get_local_orientation();
  void update(const Vector3d & accelerometer, const Vector3d & gyroscope, const double & dt);
  double invSqrt(double x);
  inline void set_gain(double gain){beta = gain;}
  inline double gain(){return beta;}

 private:
  double beta;
  double q0, q1, q2, q3;
  bool _initialize;
};

#endif  // SD_SLAM_MADGWICK_H_