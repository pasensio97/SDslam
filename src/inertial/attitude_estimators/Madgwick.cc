#include "inertial/attitude_estimators/Madgwick.h"

Madgwick::Madgwick(double gain){
    beta = gain;
    q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
    _initialize = false;
  }

const Quaterniond Madgwick::get_orientation(){
  return Quaterniond(q0, q1, q2, q3).normalized();
}

Quaterniond Madgwick::get_local_orientation(){
  Quaterniond q = get_orientation();

  Matrix3d R;  // R = Rotx(-90)*Roty(90) | x->z; y->-x; z->-y
  R << 0,  0,  1,
      -1,  0,  0,
       0, -1,  0;

  Matrix3d pose_rot;
  pose_rot = R.transpose() * q.toRotationMatrix() * R;
  Quaterniond local_q(pose_rot.inverse());
  return local_q.normalized(); // rot from NWU frame to SDSLAM local
}

void Madgwick::set_orientation(const Quaterniond & orientation){ 
  Quaterniond orientation_norm = orientation.normalized(); // Necessary?
  q0 = orientation.w();
  q1 = orientation.x();
  q2 = orientation.y();
  q3 = orientation.z();
}

void Madgwick::set_orientation_from_frame(const Matrix4d & local_pose){
  Matrix3d R;  // R = Rotx(-90)*Roty(90) | x->z; y->-x; z->-y
  R << 0,  0,  1,
      -1,  0,  0,
       0, -1,  0;

  Matrix3d pose_rot, w_pose;
  w_pose = local_pose.block<3, 3>(0, 0).transpose();
  pose_rot = R * w_pose * R.transpose();  // rot camera frame to NWU
  set_orientation(Quaterniond(pose_rot));
}

void Madgwick::update(const Vector3d & accelerometer, const Vector3d & gyroscope, const double & dt) {
  double ax = accelerometer.x();
  double ay = accelerometer.y();
  double az = accelerometer.z();
  double gx = gyroscope.x();
  double gy = gyroscope.y();
  double gz = gyroscope.z();

  double recipNorm;
  double s0, s1, s2, s3;
  double qDot1, qDot2, qDot3, qDot4;
  double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);  
  qDot2 = 0.5 * ( q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5 * ( q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5 * ( q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0 * q0;
    _2q1 = 2.0 * q1;
    _2q2 = 2.0 * q2;
    _2q3 = 2.0 * q3;
    _4q0 = 4.0 * q0;
    _4q1 = 4.0 * q1;
    _4q2 = 4.0 * q2;
    _8q1 = 8.0 * q1;
    _8q2 = 8.0 * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient descent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

double Madgwick::invSqrt(double x) {
  return 1 / sqrt(x);
}
