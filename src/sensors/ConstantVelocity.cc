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

#include "ConstantVelocity.h"

namespace SD_SLAM {

using std::vector;

ConstantVelocity::ConstantVelocity() : Sensor() {
  state_size_ = 13;
  measurement_size_ = 7;
}

ConstantVelocity::~ConstantVelocity() {
}

void ConstantVelocity::Init(Eigen::VectorXd &X, Eigen::MatrixXd &P) {
  Eigen::Vector3d x;
  Eigen::Vector4d q;
  Eigen::Vector3d v;
  Eigen::Vector3d w;

  x << 0.0, 0.0, 0.0;       // 3D Position
  q << 1.0, 0.0, 0.0, 0.0;  // Orientation quaternion
  v << 0.0, 0.0, 0.0;       // Linear velocity
  w << 0.0, 0.0, 0.0;       // Angular velocity

  X.segment<3>(0) = x;
  X.segment<4>(3) = q;
  X.segment<3>(7) = v;
  X.segment<3>(10) = w;

  P.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * Sensor::COV_X_2;
  P.block<4, 4>(3, 3) = Eigen::MatrixXd::Identity(4, 4) * Sensor::COV_Q_2;
  P.block<3, 3>(7, 7) = Eigen::MatrixXd::Identity(3, 3) * Sensor::COV_V_2;
  P.block<3, 3>(10, 10) = Eigen::MatrixXd::Identity(3, 3) * Sensor::COV_W_2;
}

void ConstantVelocity::InitState(Eigen::VectorXd &X, const Eigen::VectorXd &z) {
  X.setZero();
  X.segment<7>(0) = z.segment<7>(0); // Save pose
}

void ConstantVelocity::F(Eigen::VectorXd &X, double time) {
  Eigen::Vector3d x = X.segment<3>(0);
  Eigen::Vector4d q = X.segment<4>(3);
  Eigen::Vector3d v = X.segment<3>(7);
  Eigen::Vector3d w = X.segment<3>(10);

  // x = x + x*t
  X.segment<3>(0) = x + v*time;

  // q = q X w*t
  Eigen::Quaterniond qold(q(0), q(1), q(2), q(3));
  Eigen::Quaterniond qnew = qold * QuaternionFromAngularVelocity(w * time);
  q << qnew.w(), qnew.x(), qnew.y(), qnew.z();
  X.segment<4>(3) = q;

  // v = v
  X.segment<3>(7) = v;

  // w = w
  X.segment<3>(10) = w;
}

Eigen::MatrixXd ConstantVelocity::jF(const Eigen::VectorXd &X, double time) {
  Eigen::MatrixXd jF(state_size_, state_size_);

  Eigen::Vector4d q = X.segment<4>(3);
  Eigen::Vector3d w = X.segment<3>(10);

  // Jacobian F
  // dx/dx   dx/dq   dx/dv   dx/dw       I     0     I*t     0
  // dq/dx   dq/dq   dq/dv   dq/dw       0     dq/dq 0       dq/dw
  // dv/dx   dv/dq   dv/dv   dv/dw   =   0     0     I       0
  // dw/dx   dw/dq   dw/dv   dw/dw       0     0     0       I
  jF.setIdentity();
  jF.block<3, 3>(0, 7) = Eigen::MatrixXd::Identity(3, 3) * time;

  // dq/dq
  Eigen::Quaterniond qwt = QuaternionFromAngularVelocity(w * time);
  jF.block<4, 4>(3, 3) = QuaternionJacobian(qwt);

  // dq/dw
  Eigen::Quaterniond qold(q(0), q(1), q(2), q(3));
  jF.block<4, 3>(3, 10) = dq_by_dw(qold, w, time);

  return jF;
}

Eigen::MatrixXd ConstantVelocity::Q(const Eigen::VectorXd &X, double time) {
  Eigen::MatrixXd Q(state_size_, state_size_);
  int noise_size = 6;

  Eigen::Vector4d q = X.segment<4>(3);
  Eigen::Vector3d w = X.segment<3>(10);

  // Noise matrix
  Eigen::MatrixXd  P_n(noise_size, noise_size);
  P_n.setZero();
  P_n.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * Sensor::SIGMA_V * Sensor::SIGMA_V * time * time;
  P_n.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3) * Sensor::SIGMA_W * Sensor::SIGMA_W * time * time;

  // Jacobian G
  // dx/dv   dx/dw       I*t   0
  // dq/dv   dq/dw       0     dq/dw
  // dv/dv   dv/dw   =   I     0
  // dw/dv   dw/dw       0     I
  Eigen::MatrixXd G(state_size_, noise_size);
  G.setZero();

  G.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * time;
  G.block<3, 3>(7, 0) = Eigen::MatrixXd::Identity(3, 3);
  G.block<3, 3>(10, 3) = Eigen::MatrixXd::Identity(3, 3);

  Eigen::Quaterniond qold(q(0), q(1), q(2), q(3));
  G.block<4, 3>(3, 3) = dq_by_dw(qold, w, time);

  // Q = G * P_n * G'
  Q = G * P_n * G.transpose();

  return Q;
}

Eigen::VectorXd ConstantVelocity::Z(const Eigen::Matrix4d &pose, const vector<double> &params, double time) {
  Eigen::VectorXd Z(measurement_size_);

  assert(7 + params.size() == measurement_size_);

  Z.segment<7>(0) = PoseToVector(pose);

  return Z;
}

Eigen::VectorXd ConstantVelocity::H(const Eigen::VectorXd &X, double time) {
  Eigen::VectorXd H(measurement_size_);
  H.setZero();

  Eigen::Vector3d x = X.segment<3>(0);
  Eigen::Vector4d q = X.segment<4>(3);

  // x = x
  H.segment<3>(0) = x;

  // q = q
  H.segment<4>(3) = q;

  return H;
}

Eigen::MatrixXd ConstantVelocity::jH(const Eigen::VectorXd &X, double time) {
  Eigen::MatrixXd jH(measurement_size_, state_size_);

  // Jacobian H
  // dx/dx   dx/dq   dx/dv   dx/dw       I     0     0     0
  // dq/dx   dq/dq   dq/dv   dq/dw   =   0     I     0     0
  jH.setZero();
  jH.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
  jH.block<4, 4>(3, 3) = Eigen::MatrixXd::Identity(4, 4);

  return jH;
}

Eigen::MatrixXd ConstantVelocity::R(const Eigen::VectorXd &X, double time) {
  Eigen::MatrixXd R(measurement_size_, measurement_size_);

  R.setZero();
  R.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * Sensor::SIGMA_X * Sensor::SIGMA_X * time * time;
  R.block<4, 4>(3, 3) = Eigen::MatrixXd::Identity(4, 4) * Sensor::SIGMA_Q * Sensor::SIGMA_Q * time * time;

  return R;
}

}  // namespace SD_SLAM
