#include <Eigen/Dense>
using namespace Eigen;

class IMU_Measurements{
 public:
  IMU_Measurements();
  IMU_Measurements(const double timestamp, const Eigen::Vector3d acceleration, const Eigen::Vector3d angular_velocity);
  IMU_Measurements apply_R(const Eigen::Matrix3d& R);

  inline double timestamp(){return _timestamp;}
  inline const Eigen::Vector3d acceleration(){return _acceleration;}
  inline const Eigen::Vector3d angular_velocity() {return _angular_velocity;}


 private:
  double _timestamp;
  Eigen::Vector3d _acceleration;
  Eigen::Vector3d _angular_velocity; 

};