/**
 *
 *  Copyright (C) 2018 Eduardo Perdices <eperdices at gsyc dot es>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "IMU.h"

namespace SD_SLAM {

using std::vector;

const double IMU::COV_A_2 = 0.000625;  // 0.025^2

const double IMU::SIGMA_GYRO = 2.60; // rad/s^2
const double IMU::SIGMA_ACC = 8.94;  // m/s^3

IMU::IMU() : Sensor() {
  state_size_ = 16;
  measurement_size_ = 13;
}

IMU::~IMU() {
}

void IMU::Init(Eigen::VectorXd &X, Eigen::MatrixXd &P) {
  Eigen::Vector3d x, v, w, a;
  Eigen::Vector4d q;

  x << 0.0, 0.0, 0.0;       // 3D Position
  q << 1.0, 0.0, 0.0, 0.0;  // Orientation quaternion
  v << 0.0, 0.0, 0.0;       // Linear velocity
  w << 0.0, 0.0, 0.0;       // Angular velocity
  a << 0.0, 0.0, 0.0;       // Linear acceleration

  X.segment<3>(0) = x;
  X.segment<4>(3) = q;
  X.segment<3>(7) = v;
  X.segment<3>(10) = w;
  X.segment<3>(13) = a;

  P.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * Sensor::COV_X_2;
  P.block<4, 4>(3, 3) = Eigen::MatrixXd::Identity(4, 4) * Sensor::COV_Q_2;
  P.block<3, 3>(7, 7) = Eigen::MatrixXd::Identity(3, 3) * Sensor::COV_V_2;
  P.block<3, 3>(10, 10) = Eigen::MatrixXd::Identity(3, 3) * Sensor::COV_W_2;
  P.block<3, 3>(13, 13) = Eigen::MatrixXd::Identity(3, 3) * IMU::COV_A_2;

  gravity_.setZero();
}

void IMU::InitState(Eigen::VectorXd &X, const Eigen::VectorXd &z) {
  X.setZero();
  X.segment<7>(0) = z.segment<7>(0);  // Save pose

  gravity_.setZero();
}

void IMU::F(Eigen::VectorXd &X, double time) {
  Eigen::Vector3d x = X.segment<3>(0);
  Eigen::Vector4d q = X.segment<4>(3);
  Eigen::Vector3d v = X.segment<3>(7);
  Eigen::Vector3d w = X.segment<3>(10);
  Eigen::Vector3d a = X.segment<3>(13);

  // x = x + x*t
  X.segment<3>(0) = x + v*time;

  // q = q X w*t
  Eigen::Quaterniond qold(q(0), q(1), q(2), q(3));
  Eigen::Quaterniond qnew = qold * QuaternionFromAngularVelocity(w * time);
  q << qnew.w(), qnew.x(), qnew.y(), qnew.z();
  X.segment<4>(3) = q;

  // v = v + a*t
  X.segment<3>(7) = v + a*time;

  // w = w
  X.segment<3>(10) = w;

  // a = a
  X.segment<3>(13) = a;
}

Eigen::MatrixXd IMU::jF(const Eigen::VectorXd &X, double time) {
  Eigen::MatrixXd jF(state_size_, state_size_);

  Eigen::Vector4d q = X.segment<4>(3);
  Eigen::Vector3d w = X.segment<3>(10);

  // Jacobian F
  // dx/dx   dx/dq   dx/dv   dx/dw   dx/da       I     0     I*t   0     0
  // dq/dx   dq/dq   dq/dv   dq/dw   dq/da       0     dq/dq 0     dq/dw 0
  // dv/dx   dv/dq   dv/dv   dv/dw   dv/da   =   0     0     I     0     I*t
  // dw/dx   dw/dq   dw/dv   dw/dw   dw/da       0     0     0     I     0
  // da/dx   da/dq   da/dv   da/dw   da/da       0     0     0     0     I
  jF.setIdentity();
  jF.block<3, 3>(0, 7) = Eigen::MatrixXd::Identity(3, 3) * time;
  jF.block<3, 3>(7, 13) = Eigen::MatrixXd::Identity(3, 3) * time;

  // dq/dq
  Eigen::Quaterniond qwt = QuaternionFromAngularVelocity(w * time);
  jF.block<4, 4>(3, 3) = QuaternionJacobian(qwt);

  // dq/dw
  Eigen::Quaterniond qold(q(0), q(1), q(2), q(3));
  jF.block<4, 3>(3, 10) = dq_by_dw(qold, w, time);

  return jF;
}

Eigen::MatrixXd IMU::Q(const Eigen::VectorXd &X, double time) {
  Eigen::MatrixXd Q(state_size_, state_size_);
  int noise_size = 9;

  Eigen::Vector4d q = X.segment<4>(3);
  Eigen::Vector3d w = X.segment<3>(10);

  // Noise matrix
  Eigen::MatrixXd  P_n(noise_size, noise_size);
  P_n.setZero();
  P_n.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * Sensor::SIGMA_V * Sensor::SIGMA_V * time * time;
  P_n.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3) * Sensor::SIGMA_W * Sensor::SIGMA_W * time * time;
  P_n.block<3, 3>(6, 6) = Eigen::MatrixXd::Identity(3, 3) * IMU::SIGMA_ACC * IMU::SIGMA_ACC * time * time;

  // Jacobian G
  // dx/dv   dx/dw   dx/da       I*t   0     0
  // dq/dv   dq/dw   dq/da       0     dq/dw 0
  // dv/dv   dv/dw   dv/da   =   I     0     I*t
  // dw/dv   dw/dw   dw/da       0     I     0
  // da/dv   da/dw   da/da       0     0     I
  Eigen::MatrixXd G(state_size_, noise_size);
  G.setZero();

  G.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * time;
  G.block<3, 3>(7, 0) = Eigen::MatrixXd::Identity(3, 3);
  G.block<3, 3>(7, 6) = Eigen::MatrixXd::Identity(3, 3) * time;
  G.block<3, 3>(10, 3) = Eigen::MatrixXd::Identity(3, 3);
  G.block<3, 3>(13, 6) = Eigen::MatrixXd::Identity(3, 3);

  Eigen::Quaterniond qold(q(0), q(1), q(2), q(3));
  G.block<4, 3>(3, 3) = dq_by_dw(qold, w, time);

  // Q = G * P_n * G'
  Q = G * P_n * G.transpose();

  return Q;
}

Eigen::VectorXd IMU::Z(const Eigen::Matrix4d &pose, const vector<double> &params, double time) {
  Eigen::VectorXd Z(measurement_size_);

  assert(7 + static_cast<int>(params.size()) == measurement_size_);

  Eigen::Vector3d w(params[0], params[1], params[2]);
  Eigen::Vector3d a(params[3], params[4], params[5]);

  // Update gravity
  UpdateGravity(a, time);

  Z.segment<7>(0) = PoseToVector(pose);
  Z.segment<3>(7) = w;
  Z.segment<3>(10) = a - gravity_;

  return Z;
}

Eigen::VectorXd IMU::H(const Eigen::VectorXd &X, double time) {
  Eigen::VectorXd H(measurement_size_);
  H.setZero();

  Eigen::Vector3d x = X.segment<3>(0);
  Eigen::Vector4d q = X.segment<4>(3);
  Eigen::Vector3d w = X.segment<3>(10);
  Eigen::Vector3d a = X.segment<3>(13);

  // x = x
  H.segment<3>(0) = x;

  // q = q
  H.segment<4>(3) = q;

  // w = w
  H.segment<3>(7) = w;

  // a = a
  H.segment<3>(10) = a;

  return H;
}

Eigen::MatrixXd IMU::jH(const Eigen::VectorXd &X, double time) {
  Eigen::MatrixXd jH(measurement_size_, state_size_);

  // Jacobian H
  // dx/dx   dx/dq   dx/dv   dx/dw   dx/da       I     0     0     0     0
  // dq/dx   dq/dq   dq/dv   dq/dw   dq/da   =   0     I     0     0     0
  // dw/dx   dw/dq   dw/dv   dw/dw   dw/da       0     0     0     I     0
  // da/dx   da/dq   da/dv   da/dw   da/da       0     0     0     0     I
  jH.setZero();
  jH.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
  jH.block<4, 4>(3, 3) = Eigen::MatrixXd::Identity(4, 4);
  jH.block<3, 3>(7, 10) = Eigen::MatrixXd::Identity(3, 3);
  jH.block<3, 3>(10, 13) = Eigen::MatrixXd::Identity(3, 3);

  return jH;
}

Eigen::MatrixXd IMU::R(const Eigen::VectorXd &X, double time) {
  Eigen::MatrixXd R(measurement_size_, measurement_size_);

  R.setZero();
  R.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * Sensor::SIGMA_X * Sensor::SIGMA_X * time * time;
  R.block<4, 4>(3, 3) = Eigen::MatrixXd::Identity(4, 4) * Sensor::SIGMA_Q * Sensor::SIGMA_Q * time * time;
  R.block<3, 3>(7, 7) = Eigen::MatrixXd::Identity(3, 3) * IMU::SIGMA_GYRO * IMU::SIGMA_GYRO * time * time;
  R.block<3, 3>(10, 10) = Eigen::MatrixXd::Identity(3, 3) * IMU::SIGMA_ACC * IMU::SIGMA_ACC * time * time;

  return R;
}

void IMU::UpdateGravity(const Eigen::Vector3d &a, double time) {
  // Low pass filter
  double alpha =  0.27 / (0.27 + time);

  gravity_(0) = alpha * gravity_(0) + (1 - alpha) * a(0);
  gravity_(1) = alpha * gravity_(1) + (1 - alpha) * a(1);
  gravity_(2) = alpha * gravity_(2) + (1 - alpha) * a(2);
}

}  // namespace SD_SLAM
