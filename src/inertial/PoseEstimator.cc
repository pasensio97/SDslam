
#include "PoseEstimator.h"
#include <Eigen/Dense>
#include <iostream>

const double COV_A_2 = 0.000625;  // 0.025^2
const double SIGMA_GYRO = 2.60; // rad/s^2
const double SIGMA_ACC = 8.94;  // m/s^3

PoseEstimator::PoseEstimator() {
  dim_state_vector = 9;
  dim_measurements = 3;

  initilized = false;

  x = Eigen::MatrixXd::Zero(dim_state_vector, 1);  
  z = Eigen::MatrixXd::Zero(dim_measurements, 1);

  F = Eigen::MatrixXd::Identity(dim_state_vector, dim_state_vector);
  P = Eigen::MatrixXd::Identity(dim_state_vector, dim_state_vector);  // **
  Q = Eigen::MatrixXd::Identity(dim_state_vector, dim_state_vector);  // **

  y = Eigen::MatrixXd::Zero(dim_measurements, 1);
  H = Eigen::MatrixXd::Identity(dim_measurements, dim_state_vector);
  R = Eigen::MatrixXd::Identity(dim_measurements, dim_measurements) * 10;  // **
  S = Eigen::MatrixXd::Identity(dim_measurements, dim_measurements);
  K = Eigen::MatrixXd::Identity(dim_measurements, dim_measurements);

  Id = Eigen::MatrixXd::Identity(dim_state_vector, dim_state_vector);
}


PoseEstimator::~PoseEstimator() {
}


Eigen::VectorXd PoseEstimator::predict(double dt){
  this->dt = dt;

  F = jacobian_f(x, dt);
  
  x = f(x, dt);
  P = F * P * F.transpose() + Q;

  return state_vector();
}

void PoseEstimator::update(const Eigen::VectorXd & measurements){
  z = measurements;
  
  y = z - h(x, dt);
  H = jacobian_h(x, dt);
  S = H * P * H.transpose() + R;
  K = P * H.transpose() * S.inverse();

  x = x + K * y;
  P = (Id - K * H) * P;
}

const Eigen::VectorXd PoseEstimator::state_vector(){
  return x;
}


Eigen::Vector3d PoseEstimator::get_position(){
  return x.segment<3>(0);
}


void PoseEstimator::print_state_vector(){
  Eigen::Vector3d pos = x.segment<3>(0);
  Eigen::Vector3d vel = x.segment<3>(3);
  Eigen::Vector3d acc = x.segment<3>(6);
  std::cout << "State vector value:\n" 
            << "\tPos: (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")\n" 
            << "\tVel: (" << vel.x() << ", " << vel.y() << ", " << vel.z() << ")\n" 
            << "\tAcc: (" << acc.x() << ", " << acc.y() << ", " << acc.z() << ")\n" 
            <<  std::endl;
}

Eigen::VectorXd PoseEstimator::f(Eigen::VectorXd & x, double dt){
  Eigen::Vector3d pos = x.segment<3>(0);
  Eigen::Vector3d vel = x.segment<3>(3);
  Eigen::Vector3d acc = x.segment<3>(6);

  x.segment<3>(0) = pos + vel*dt + 0.5 * acc * dt*dt;
  x.segment<3>(3) =       vel +          acc * dt;
  x.segment<3>(6) =                      acc;
  return x;
}

Eigen::MatrixXd PoseEstimator::jacobian_f(Eigen::VectorXd & x, double dt){
  /*
    |  1     dt   acc*dt  |
    |  0     1     dt     |
    |  0     0     1      |
  */
  Eigen::Vector3d acc = x.segment<3>(6);
  /*
  F = Eigen::MatrixXd::Identity(dim_state_vector, dim_state_vector);
  F.block<3,3>(0,3) = Eigen::MatrixXd::Identity(3,3) * dt;
  F.block<3,3>(0,6) = Eigen::MatrixXd::Identity(3,3) * x.block<1,3>(0,6) * dt;
  F.block<3,3>(3,6) = Eigen::MatrixXd::Identity(3,3) * dt;
  */
  F << 1,  0,  0,     dt,   0,   0,    acc.x()*dt,      0,           0,
        0,  1,  0,     0,   dt,   0,        0,       acc.y()*dt,      0,
        0,  0,  1,     0,   0,   dt,        0,           0,       acc.z()*dt,

        0,  0,  0,     1,   0,   0,     dt,   0,   0,
        0,  0,  0,     0,   1,   0,     0,   dt,   0,
        0,  0,  0,     0,   0,   1,     0,   0,   dt,

        0,  0,  0,     0,   0,   0,     1,   0,   0,
        0,  0,  0,     0,   0,   0,     0,   1,   0,
        0,  0,  0,     0,   0,   0,     0,   0,   1;

  return F;
}

Eigen::VectorXd PoseEstimator::h(const Eigen::VectorXd & x, double dt){
  /* 
    h(x,k) = [ 0, 0, acc ] 
  */
  Eigen::MatrixXd h(1, dim_state_vector);
  Eigen::Vector3d acc = x.block<1,3>(0,6);
  h << 0, 0, 0,   0, 0, 0,   acc.x(), acc.y(), acc.z();
  return h;
}

Eigen::MatrixXd PoseEstimator::jacobian_h(Eigen::VectorXd & x, double dt){
  /* 
    H = [ 0, 0, 1 ] 
  */
  Eigen::MatrixXd jH = Eigen::MatrixXd::Zero(dim_measurements, dim_state_vector);
  jH.block<3,3>(0,6) = Eigen::MatrixXd::Identity(dim_measurements, dim_measurements);
  return jH;
}

  
