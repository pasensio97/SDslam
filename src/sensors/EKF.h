/**
 *
 *  Copyright (C) 2017 Eduardo Perdices <eperdices at gsyc dot es>
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

#ifndef SD_SLAM_EKF_H_
#define SD_SLAM_EKF_H_

#include <iostream>
#include <Eigen/Dense>
#include "Sensor.h"
#include "extra/timer.h"

namespace SD_SLAM {

class EKF {
 public:
  EKF(Sensor *sensor);
  ~EKF();

  inline bool Started() { return updated_; }

  // Predict EKF and return 3D Pose
  Eigen::Matrix4d Predict(const Eigen::Matrix4d &pose);

  // Update EKF
  void Update(const Eigen::Matrix4d &pose, const std::vector<double> &params);

  // Restart filter
  void Restart();

 private:
  Sensor* sensor_;    // Motion sensor

  bool updated_;      // True if EKF has been updated at least once
  Timer timer_;       // Measure time since last iteration
  double it_time_;    // Time (s) since last iteration

  // State and covariance
  Eigen::VectorXd X_;
  Eigen::MatrixXd P_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_EKF_H_
