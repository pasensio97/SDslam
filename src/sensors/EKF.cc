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

#include "EKF.h"

using std::vector;

namespace SD_SLAM {

EKF::EKF(Sensor *sensor) : timer_(false) {
  sensor_ = sensor;
  it_time_ = 0.0;

  const int size_s = sensor_->GetStateSize();

  X_.resize(size_s, 1);
  X_.setZero();
  P_.resize(size_s, size_s);
  P_.setZero();

  updated_ = false;
  sensor_->Init(X_, P_);
}

EKF::~EKF() {
}

Eigen::Matrix4d EKF::Predict() {
  if (updated_) {
    timer_.Stop();
    it_time_ = timer_.GetTime();
  } else {
    it_time_ = 0.0;
  }

  // Get matrices before predict
  Eigen::MatrixXd jF = sensor_->jF(X_, it_time_);
  Eigen::MatrixXd Q = sensor_->Q(X_, it_time_);

  // X_0 = f(X_0)
  sensor_->F(X_, it_time_);

  // P_0 = jF * P_0 * jF' + Q
  P_ = jF * P_ * jF.transpose() + Q;

  return sensor_->GetPose(X_);
}

void EKF::Update(const Eigen::Matrix4d &pose, const vector<double> &params) {
  Eigen::VectorXd Y;
  Eigen::MatrixXd S, K;

  // Set measurements vector
  Eigen::VectorXd Z = sensor_->Z(pose, params, it_time_);

  if (!updated_) {
    // Set initial state
    sensor_->InitState(X_, Z);
  } else {
    // Get matrices before update
    Eigen::VectorXd H = sensor_->H(X_, it_time_);
    Eigen::MatrixXd jH = sensor_->jH(X_, it_time_);
    Eigen::MatrixXd R = sensor_->R(X_, it_time_);

    // Y = Z - h(X_0)
    Y = Z - H;

    // S = jH * P_0 * jH' + R
    S = jH * P_ * jH.transpose() + R;

    // K = P_0 * jH' * S^-1
    K = P_ * jH.transpose() * S.inverse();

    // X = X_0 + K * Y
    X_ = X_ + K * Y;

    // P = P_0 - K * S * K'
    P_ = P_ - K * S *  K.transpose();
    //P_ = 0.5 * P_ + 0.5 * P_.transpose().eval(); //Enforce covariance symmetry
  }

  // Start timer for next iteration
  updated_ = true;
  timer_.Start();
}

void EKF::Restart() {
  updated_ = false;
  sensor_->Init(X_, P_);
}

}  // namespace SD_SLAM
