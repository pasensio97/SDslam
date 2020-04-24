/*
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  Based on implementation of Madgwick's IMU and AHRS algorithms.
 *  http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SD_SLAM_MADGWICK_ROS_H_
#define SD_SLAM_MADGWICK_ROS_H_

#include <math.h>
#include <Eigen/Dense>

using namespace Eigen;

class ImuFilter
{
  public:

    ImuFilter();
    ImuFilter(double gain, double zeta=0.0);
    virtual ~ImuFilter();

  private:
    // **** paramaters
    double gain_;    // algorithm gain
    double zeta_;    // gyro drift bias gain

    // **** state variables
    double q0, q1, q2, q3;  // quaternion
public:
    void setAlgorithmGain(double gain)
    {
      gain_ = gain;
    }

    void setDriftBiasGain(double zeta)
    {
      zeta_ = zeta;
    }

    void getOrientation(double& q0, double& q1, double& q2, double& q3)
    {
        q0 = this->q0;
        q1 = this->q1;
        q2 = this->q2;
        q3 = this->q3;

        // perform precise normalization of the output, using 1/sqrt()
        // instead of the fast invSqrt() approximation. Without this,
        // TF2 complains that the quaternion is not normalized.
        double recipNorm = 1 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    void setOrientation(double q0, double q1, double q2, double q3)
    {
        this->q0 = q0;
        this->q1 = q1;
        this->q2 = q2;
        this->q3 = q3;
    }

    void set_orientation(Eigen::Quaterniond orientation){
      orientation.normalize(); // Necessary?
      this->q0 = orientation.w();
      this->q0 = orientation.x();
      this->q2 = orientation.y();
      this->q3 = orientation.z();
    }

    Eigen::Quaterniond get_orientation(){
        double qw = this->q0;
        double qx = this->q1;
        double qy = this->q2;
        double qz = this->q3;

        // perform precise normalization of the output, using 1/sqrt()
        // instead of the fast invSqrt() approximation. Without this,
        // TF2 complains that the quaternion is not normalized.
        double recipNorm = 1 / sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        qw *= recipNorm;
        qx *= recipNorm;
        qy *= recipNorm;
        qz *= recipNorm;

        return Eigen::Quaterniond(qw, qx, qy, qz);
    }

    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float dt);
};

#endif // SD_SLAM_MADGWICK_ROS_H_
