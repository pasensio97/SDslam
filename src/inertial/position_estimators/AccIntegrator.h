

#ifndef SD_SLAM_ACCINTEGRATOR_POSITION_H_
#define SD_SLAM_ACCINTEGRATOR_POSITION_H_

#include <Eigen/Dense>
using namespace Eigen;

class AccIntegrator{
 private:
  const Vector3d GRAVITY = Vector3d(0.0, 0.0, 9.80665);
  bool _acc_due_gravity;
  Vector3d _gravity;


 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Vector3d position;
  Vector3d velocity;

  AccIntegrator(bool acc_due_gravity);
  Vector3d update(const Vector3d & acceleration, const Quaterniond & sensor_attitude, const double & dt);
  Vector3d remove_gravity(const Vector3d & acceleration, const Quaterniond & sensor_attitude);

  inline void set_custom_gravity(const Vector3d & gravity){_gravity = gravity;};
};

#endif  // SD_SLAM_ACCINTEGRATOR_POSITION_H_