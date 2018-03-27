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

#ifndef SD_SLAM_CONSTANT_VELOCITY_H_
#define SD_SLAM_CONSTANT_VELOCITY_H_

#include <iostream>
#include <vector>
#include "Sensor.h"

namespace SD_SLAM {

const double SMALL_EPS = 1e-10;

class ConstantVelocity : public Sensor {
 public:
  ConstantVelocity();
  ~ConstantVelocity();

  void Init(Eigen::VectorXd &X, Eigen::MatrixXd &P);
  void InitState(Eigen::VectorXd &X, const Eigen::VectorXd &Z);

  Eigen::Matrix4d GetPose(const Eigen::VectorXd &X);
  Eigen::VectorXd PoseToVector(const Eigen::Matrix4d &pose);

  void F(Eigen::VectorXd &X, double time);
  Eigen::MatrixXd jF(const Eigen::VectorXd &X, double time);
  Eigen::MatrixXd Q(const Eigen::VectorXd &X, double time);

  Eigen::VectorXd Z(const Eigen::Matrix4d &pose, const std::vector<double> &params, double time);
  Eigen::VectorXd H(const Eigen::VectorXd &X, double time);
  Eigen::MatrixXd jH(const Eigen::VectorXd &X, double time);
  Eigen::MatrixXd R(const Eigen::VectorXd &X, double time);

 private:
  Eigen::Matrix4d Exp(const Eigen::Matrix<double, 6, 1> &update);
  Eigen::Matrix<double, 6, 1> Log(const Eigen::Matrix4d &pose);
  Eigen::Quaterniond RotationExp(const Eigen::Vector3d &omega, double *theta);
  Eigen::Matrix3d RotationHat(const Eigen::Vector3d &v);
  Eigen::Vector3d RotationLog(const Eigen::Quaterniond &other, double *theta);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_CONSTANT_VELOCITY_H_
