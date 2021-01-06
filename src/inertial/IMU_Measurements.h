
#ifndef SD_SLAM_IMUMEASUREMENTS_H_
#define SD_SLAM_IMUMEASUREMENTS_H_

#include <Eigen/Dense>
using namespace Eigen;

class IMU_Measurements{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  IMU_Measurements();
  IMU_Measurements(const double & timestamp, const Eigen::Vector3d & acceleration, const Eigen::Vector3d & angular_velocity);
  IMU_Measurements apply_R(const Eigen::Matrix3d& R);

  inline double timestamp(){return _timestamp;}
  inline Eigen::Vector3d acceleration() const {return Vector3d(_acceleration);}
  inline Eigen::Vector3d angular_velocity() const {return Vector3d(_angular_velocity);}


 private:
  double _timestamp;
  Eigen::Vector3d _acceleration;
  Eigen::Vector3d _angular_velocity; 

};

#endif  // SD_SLAM_IMUMEASUREMENTS_H_