
#include <math.h>
#include <Eigen/Dense>

using namespace Eigen;

class Madgwick{
  // based on http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 public:
  Madgwick(double gain);

  void set_orientation(Quaterniond orientation);
  const Quaterniond get_orientation();
  void update(Vector3d accelerometer, Vector3d gyroscope, double dt);
  double invSqrt(double x);

 private:
  double beta;
  double q0, q1, q2, q3;
  bool _initialize;
};