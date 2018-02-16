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

#include "Sensor.h"

namespace SD_SLAM {

const double Sensor::COV_X_2 = 0.0025;    // 0.05^2
const double Sensor::COV_Q_2 = 0.00001;   // 0.001^2
const double Sensor::COV_V_2 = 0.000625;  // 0.025^2
const double Sensor::COV_W_2 = 0.000625;  // 0.025^2

const double Sensor::SIGMA_X = 0.05;  // m/s
const double Sensor::SIGMA_Q = 0.02;  // rad/s
const double Sensor::SIGMA_V = 4.0;   // m/s^2
const double Sensor::SIGMA_W = 6.0;   // rad/s^2

Sensor::Sensor() {
  state_size_ = 0;
  measurement_size_ = 0;
}

Sensor::~Sensor() {
}

Eigen::Matrix4d Sensor::GetPose(const Eigen::VectorXd &X) {
  Eigen::Matrix4d pose;

  Eigen::Vector3d x = X.segment<3>(0);
  Eigen::Vector4d q = X.segment<4>(3);

  Eigen::Quaterniond qt(q(0), q(1), q(2), q(3));
  qt.normalize();

  pose.setIdentity();
  pose.block<3, 3>(0, 0) = qt.toRotationMatrix();
  pose.block<3, 1>(0, 3) = x;

  return pose;
}

Eigen::VectorXd Sensor::PoseToVector(const Eigen::Matrix4d &pose) {
  Eigen::VectorXd v(7);

  Eigen::Matrix3d rot = pose.block<3, 3>(0, 0);
  Eigen::Quaterniond q(rot);
  q.normalize();

  v.segment<3>(0) = pose.block<3, 1>(0, 3);
  v(3) = q.w();
  v(4) = q.x();
  v(5) = q.y();
  v(6) = q.z();

  return v;
}

Eigen::Quaterniond Sensor::QuaternionFromAngularVelocity(const Eigen::Vector3d &w) {
  Eigen::Quaterniond  q;
  const double angle = sqrt(w(0)*w(0) + w(1)*w(1) + w(2)*w(2));

  if (angle > 0.0) {
    const double s = sin(angle/2.0) / angle;
    const double c = cos(angle/2.0);

    q.w() = c;
    q.x() = s * w(0);
    q.y() = s * w(1);
    q.z() = s * w(2);
  } else {
    q.w() = 1.0;
    q.x() = q.y() = q.z() = 0.0;
  }

  return  q;
}

Eigen::Matrix4d Sensor::QuaternionJacobian(const Eigen::Quaterniond &q) {
  Eigen::Matrix4d m;

  double  x = q.x();
  double  y = q.y();
  double  z = q.z();
  double  w = q.w();

  m << w, -x, -y, -z,
       x,  w,  z, -y,
       y, -z,  w,  x,
       z,  y, -x,  w;

  return m;
}

Eigen::Matrix4d Sensor::QuaternionJacobianRight(const Eigen::Quaterniond &q) {
  Eigen::Matrix4d m;

  double  x = q.x();
  double  y = q.y();
  double  z = q.z();
  double  w = q.w();

  m << w, -x, -y, -z,
       x,  w, -z,  y,
       y,  z,  w, -x,
       z, -y,  x,  w;

  return m;
}

Eigen::Matrix<double, 4, 3> Sensor::dq_by_dw(const Eigen::Quaterniond &q, const Eigen::Vector3d &w, double time) {
  double modw = w.norm();
  double beta = modw * time / 2.0;
  Eigen::Matrix<double, 4, 3> res;
  Eigen::Matrix<double, 4, 3> mdw;

  if (modw == 0) {
    res.block(0, 0, 1, 3).setZero();
    res.block(1, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * time / 2.0;
  } else {
    mdw(0, 0) = (-time / 2.0) * sin(beta) * w(0) / modw;
    mdw(0, 1) = (-time / 2.0) * sin(beta) * w(1) / modw;
    mdw(0, 2) = (-time / 2.0) * sin(beta) * w(2) / modw;

    mdw(1, 0) = (time / 2.0) * cos(beta) * SQUARE(w(0)) / SQUARE(modw) + sin(beta) / modw * (1.0 - SQUARE(w(0)) / SQUARE(modw));
    mdw(1, 1) = (w(0) * w(1) / (SQUARE(modw))) * ((time / 2.0) * cos(beta) - sin(beta) / modw);
    mdw(1, 2) = (w(0) * w(2) / (SQUARE(modw))) * ((time / 2.0) * cos(beta) - sin(beta) / modw);

    mdw(2, 0) = (w(1) * w(0) / (SQUARE(modw))) * ((time / 2.0) * cos(beta) - sin(beta) / modw);
    mdw(2, 1) = (time / 2.0) * cos(beta) * SQUARE(w(1)) / SQUARE(modw) + sin(beta) / modw * (1.0 - SQUARE(w(1)) / SQUARE(modw));
    mdw(2, 2) = (w(1) * w(2) / (SQUARE(modw))) * ((time / 2.0) * cos(beta) - sin(beta) / modw);

    mdw(3, 0) = (w(2) * w(0) / (SQUARE(modw))) * ((time / 2.0) * cos(beta) - sin(beta) / modw);
    mdw(3, 1) = (w(2) * w(1) / (SQUARE(modw))) * ((time / 2.0) * cos(beta) - sin(beta) / modw);
    mdw(3, 2) = (time / 2.0) * cos(beta) * SQUARE(w(2)) / SQUARE(modw) + sin(beta) / modw * (1.0 - SQUARE(w(2)) / SQUARE(modw));

    /*Calculate quaternion jacobian*/
    Eigen::Matrix4d mjac_right = QuaternionJacobianRight(q);
    res = mjac_right * mdw;
  }

  return res;
}

}  // namespace SD_SLAM
