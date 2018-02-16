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

#ifndef SD_SLAM_IMU_H_
#define SD_SLAM_IMU_H_

#include <iostream>
#include <vector>
#include "Sensor.h"

namespace SD_SLAM {

class IMU : public Sensor {
 public:
  IMU();
  ~IMU();

  void Init(Eigen::VectorXd &X, Eigen::MatrixXd &P);
  void InitState(Eigen::VectorXd &X, const Eigen::VectorXd &Z);

  void F(Eigen::VectorXd &X, double time);
  Eigen::MatrixXd jF(const Eigen::VectorXd &X, double time);
  Eigen::MatrixXd Q(const Eigen::VectorXd &X, double time);

  Eigen::VectorXd Z(const Eigen::Matrix4d &pose, const std::vector<double> &params, double time);
  Eigen::VectorXd H(const Eigen::VectorXd &X, double time);
  Eigen::MatrixXd jH(const Eigen::VectorXd &X, double time);
  Eigen::MatrixXd R(const Eigen::VectorXd &X, double time);

 private:
  // Calculate gravity from IMU
  void UpdateGravity(const Eigen::Vector3d &a, double time);

  // Gravity parameters
  Eigen::Vector3d gravity_;

  // Covariance
  static const double COV_A_2;

  // Noise
  static const double SIGMA_GYRO;
  static const double SIGMA_ACC;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_IMU_H_
