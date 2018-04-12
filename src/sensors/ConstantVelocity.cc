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
  state_size_ = 6;
  measurement_size_ = 6;
}

ConstantVelocity::~ConstantVelocity() {
}

void ConstantVelocity::Init(Eigen::VectorXd &X, Eigen::MatrixXd &P) {
  Eigen::Vector3d v;
  Eigen::Vector3d w;

  v << 0.0, 0.0, 0.0;       // Linear velocity
  w << 0.0, 0.0, 0.0;       // Angular velocity

  X.segment<3>(0) = v;
  X.segment<3>(3) = w;

  P.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * Sensor::COV_V_2;
  P.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(4, 4) * Sensor::COV_W_2;
}

void ConstantVelocity::InitState(Eigen::VectorXd &X, const Eigen::VectorXd &z) {
  X.setZero();
}

Eigen::Matrix4d ConstantVelocity::GetPose(const Eigen::VectorXd &X) {
  Eigen::Matrix<double, 6, 1> vel = X.segment<6>(0);
  return Exp(vel) * last_pose_;
}

Eigen::VectorXd ConstantVelocity::PoseToVector(const Eigen::Matrix4d &pose) {
  return Log(pose);
}

void ConstantVelocity::F(Eigen::VectorXd &X, double time) {
  Eigen::Vector3d v = X.segment<3>(0);
  Eigen::Vector3d w = X.segment<3>(3);

  // v = v
  X.segment<3>(0) = v;

  // w = w
  X.segment<3>(3) = w;
}

Eigen::MatrixXd ConstantVelocity::jF(const Eigen::VectorXd &X, double time) {
  Eigen::MatrixXd jF(state_size_, state_size_);

  // Jacobian F
  // dv/dv   dv/dw  =   I       0
  // dw/dv   dw/dw      0       I
  jF.setIdentity();

  return jF;
}

Eigen::MatrixXd ConstantVelocity::Q(const Eigen::VectorXd &X, double time) {
  Eigen::MatrixXd Q(state_size_, state_size_);
  int noise_size = 6;

  // Noise matrix
  Eigen::MatrixXd  P_n(noise_size, noise_size);
  P_n.setZero();
  P_n.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * Sensor::SIGMA_V * Sensor::SIGMA_V * time * time;
  P_n.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3) * Sensor::SIGMA_W * Sensor::SIGMA_W * time * time;

  // Jacobian G
  // dv/dv   dv/dw   =   I     0
  // dw/dv   dw/dw       0     I
  Eigen::MatrixXd G(state_size_, noise_size);
  G.setIdentity();

  // Q = G * P_n * G'
  Q = G * P_n * G.transpose();

  return Q;
}

Eigen::VectorXd ConstantVelocity::Z(const Eigen::Matrix4d &pose, const vector<double> &params, double time) {
  Eigen::VectorXd Z(measurement_size_);

  assert(6 + static_cast<int>(params.size()) == measurement_size_);

  // Get last pose inverse
  Eigen::Matrix4d last_pose_i;
  Eigen::Matrix3d rot = last_pose_.block<3, 3>(0, 0).transpose();

  last_pose_i.setIdentity();
  last_pose_i.block<3, 3>(0, 0) = rot;
  last_pose_i.block<3, 1>(0, 3) = -rot*last_pose_.block<3, 1>(0, 3);

  Eigen::Matrix4d se3 = pose * last_pose_i;
  Z.segment<6>(0) = PoseToVector(se3);

  return Z;
}

Eigen::VectorXd ConstantVelocity::H(const Eigen::VectorXd &X, double time) {
  Eigen::VectorXd H(measurement_size_);
  H.setZero();

  Eigen::Vector3d v = X.segment<3>(0);
  Eigen::Vector3d w = X.segment<3>(3);

  // v = v
  H.segment<3>(0) = v;

  // w = w
  H.segment<3>(3) = w;

  return H;
}

Eigen::MatrixXd ConstantVelocity::jH(const Eigen::VectorXd &X, double time) {
  Eigen::MatrixXd jH(measurement_size_, state_size_);

  // Jacobian H
  // dv/dv   dv/dw       I     0
  // dw/dv   dw/dw   =   0     I
  jH.setIdentity();

  return jH;
}

Eigen::MatrixXd ConstantVelocity::R(const Eigen::VectorXd &X, double time) {
  Eigen::MatrixXd R(measurement_size_, measurement_size_);

  R.setZero();
  R.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * Sensor::SIGMA_V * Sensor::SIGMA_V * time * time;
  R.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(4, 4) * Sensor::SIGMA_W * Sensor::SIGMA_W * time * time;

  return R;
}

Eigen::Matrix4d ConstantVelocity::Exp(const Eigen::Matrix<double, 6, 1> &update) {
  Eigen::Vector3d upsilon = update.head<3>();
  Eigen::Vector3d omega = update.tail<3>();
  Eigen::Matrix4d pose;

  double theta;
  Eigen::Quaterniond q = RotationExp(omega, &theta);
  Eigen::Matrix3d Omega = RotationHat(omega);
  Eigen::Matrix3d Omega_sq = Omega*Omega;
  Eigen::Matrix3d V;

  if (theta < SMALL_EPS) {
    V = q.toRotationMatrix();
    // Note: That is an accurate expansion!
  } else {
    double theta_sq = theta*theta;
    V = (Eigen::Matrix3d::Identity()
         + (1-cos(theta))/(theta_sq)*Omega
         + (theta-sin(theta))/(theta_sq*theta)*Omega_sq);
  }

  Eigen::Vector3d t = V*upsilon;

  pose.setIdentity();
  q.normalize();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose.block<3, 1>(0, 3) = t;
  return pose;
}

Eigen::Matrix<double, 6, 1> ConstantVelocity::Log(const Eigen::Matrix4d &pose) {
  Eigen::Matrix<double, 6, 1> upsilon_omega;
  double theta;

  Eigen::Matrix3d rot = pose.block<3, 3>(0, 0);
  Eigen::Quaterniond q(rot);
  q.normalize();
  upsilon_omega.tail<3>() = RotationLog(q, &theta);

  if (theta < SMALL_EPS) {
    Eigen::Matrix3d Omega = RotationHat(upsilon_omega.tail<3>());
    Eigen::Matrix3d V_inv = Eigen::Matrix3d::Identity()- 0.5*Omega + (1./12.)*(Omega*Omega);

    upsilon_omega.head<3>() = V_inv*pose.block<3, 1>(0, 3);
  } else {
    Eigen::Matrix3d Omega = RotationHat(upsilon_omega.tail<3>());
    Eigen::Matrix3d V_inv = (Eigen::Matrix3d::Identity() - 0.5*Omega + (1-theta/(2*tan(theta/2)))/(theta*theta)*(Omega*Omega));
    upsilon_omega.head<3>() = V_inv*pose.block<3, 1>(0, 3);
  }
  return upsilon_omega;
}

Eigen::Quaterniond ConstantVelocity::RotationExp(const Eigen::Vector3d &omega, double *theta) {
  *theta = omega.norm();
  double half_theta = 0.5*(*theta);

  double imag_factor;
  double real_factor = cos(half_theta);
  if ((*theta) < SMALL_EPS) {
    double theta_sq = (*theta)*(*theta);
    double theta_po4 = theta_sq*theta_sq;
    imag_factor = 0.5-0.0208333*theta_sq+0.000260417*theta_po4;
  } else {
    double sin_half_theta = sin(half_theta);
    imag_factor = sin_half_theta/(*theta);
  }

  return Eigen::Quaterniond(real_factor, imag_factor*omega.x(), imag_factor*omega.y(), imag_factor*omega.z());
}

Eigen::Matrix3d ConstantVelocity::RotationHat(const Eigen::Vector3d &v) {
  Eigen::Matrix3d Omega;
  Omega <<  0, -v(2),  v(1)
      ,  v(2),     0, -v(0)
      , -v(1),  v(0),     0;
  return Omega;
}

Eigen::Vector3d ConstantVelocity::RotationLog(const Eigen::Quaterniond &other, double *theta) {
  double n = other.vec().norm();
  double w = other.w();
  double squared_w = w*w;
  double two_atan_nbyw_by_n;

  if (n < SMALL_EPS) {
    // If quaternion is normalized and n=1, then w should be 1;
    // w=0 should never happen here!
    assert(fabs(w) > SMALL_EPS);
    two_atan_nbyw_by_n = 2./w - 2.*(n*n)/(w*squared_w);
  } else  {
    if (fabs(w) < SMALL_EPS) {
      if (w > 0) {
        two_atan_nbyw_by_n = M_PI/n;
      } else {
        two_atan_nbyw_by_n = -M_PI/n;
      }
    }
    two_atan_nbyw_by_n = 2*atan(n/w)/n;
  }

  *theta = two_atan_nbyw_by_n*n;
  return two_atan_nbyw_by_n * other.vec();
}

}  // namespace SD_SLAM
