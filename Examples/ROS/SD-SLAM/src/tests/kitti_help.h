
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

struct IMUSensor{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double time;
  Vector3d oxts_units;
  Vector3d euler_angles;
  Vector3d acc_vehicle;
  Vector3d acc_sensor;
  Vector3d angular_vel_vehicle;
  Vector3d angular_vel_sensor;
};

class Kitti{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Matrix3d rotation_imu_velo();
  Matrix3d rotation_velo_cam();
  Matrix3d rotation_imu_cam();
  void read_gps(string & filename, vector<Vector3d> &pos, vector<Quaterniond> &att);
  vector<string> load_imu_filenames(const string & filepath);
  vector<string> load_image_filenames(const string & filepath);


  /**
 * 
 0 lat:   latitude of the oxts-unit (deg)
 1 lon:   longitude of the oxts-unit (deg)
 2 alt:   altitude of the oxts-unit (m)
 3 roll:  roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
 4 pitch: pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
 5 yaw:   heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
 6 vn:    velocity towards north (m/s)
 7 ve:    velocity towards east (m/s)
 8 vf:    forward velocity, i.e. parallel to earth-surface (m/s)
 9 vl:    leftward velocity, i.e. parallel to earth-surface (m/s)
 10 vu:    upward velocity, i.e. perpendicular to earth-surface (m/s)
 11 ax:    acceleration in x, i.e. in direction of vehicle front (m/s^2)
 12 ay:    acceleration in y, i.e. in direction of vehicle left (m/s^2)
 13 az:    acceleration in z, i.e. in direction of vehicle top (m/s^2)
 14 af:    forward acceleration (m/s^2)
 15 al:    leftward acceleration (m/s^2)
 16 au:    upward acceleration (m/s^2)
 17 wx:    angular rate around x (rad/s)
 18 wy:    angular rate around y (rad/s)
 19 wz:    angular rate around z (rad/s)
 20 wf:    angular rate around forward axis (rad/s)
 21 wl:    angular rate around leftward axis (rad/s)
 22 wu:    angular rate around upward axis (rad/s)
 23 pos_accuracy:  velocity accuracy (north/east in m)
 24 vel_accuracy:  velocity accuracy (north/east in m/s)
 25 navstat:       navigation status (see navstat_to_string)
 26 numsats:       number of satellites tracked by primary GPS receiver
 27 posmode:       position mode of primary GPS receiver (see gps_mode_to_string)
 28 velmode:       velocity mode of primary GPS receiver (see gps_mode_to_string)
 29 orimode:       orientation mode of primary GPS receiver (see gps_mode_to_string)
*/
  IMUSensor read_imu_file(const string & filename);


 private:
  inline bool _file_exists (const std::string& name) {ifstream f(name.c_str()); return f.good();
}
};