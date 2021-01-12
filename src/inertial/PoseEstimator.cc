
#include "PoseEstimator.h"
#include <iostream>

const double COV_POS_2 = 0.05;
const double COV_VEL_2 = 0.001;
const double COV_ACC_2 = 0.02;  // 0.025^2

// sigma process noise
const double SIGMA_POS = 0.002;  // m
const double SIGMA_VEL = 0.006;   // m/s
const double SIGMA_ACC = 0.0001;  // m/s^2
// sigma meauses noise
const double SIGMA_ACC_MEASURE =  0.02;  // m/s^2


PoseEstimator::PoseEstimator() {
  dim_state_vector = 9;  // px, py, pz, vx, vy, vz, ax, ay, az
  dim_measurements = 3;  // ax, ay, az

  initilized = false;

  x = Eigen::MatrixXd::Zero(dim_state_vector, 1);  
  z = Eigen::MatrixXd::Zero(dim_measurements, 1);

  F = Eigen::MatrixXd::Identity(dim_state_vector, dim_state_vector);
  P = Eigen::MatrixXd::Identity(dim_state_vector, dim_state_vector);  
  Q = Eigen::MatrixXd::Identity(dim_state_vector, dim_state_vector);  

  y = Eigen::MatrixXd::Zero(dim_measurements, 1);
  H = Eigen::MatrixXd::Identity(dim_measurements, dim_state_vector);
  R = Eigen::MatrixXd::Identity(dim_measurements, dim_measurements);  
  S = Eigen::MatrixXd::Identity(dim_measurements, dim_measurements);
  K = Eigen::MatrixXd::Identity(dim_measurements, dim_measurements);

  //Id = Eigen::MatrixXd::Identity(dim_state_vector, dim_state_vector);


  P.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * COV_POS_2;
  P.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3) * COV_VEL_2;
  P.block<3, 3>(6, 6) = Eigen::MatrixXd::Identity(3, 3) * COV_ACC_2;

}




Eigen::VectorXd PoseEstimator::predict(double dt){
  this->dt = dt;

  jacobian_f(x, dt);
  f(x, dt);
  //test with Q
  Q.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * SIGMA_POS * dt * dt;
  Q.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3) * SIGMA_VEL * dt * dt;
  Q.block<3, 3>(6, 6) = Eigen::MatrixXd::Identity(3, 3) * SIGMA_ACC * dt * dt;
  P = F * P * F.transpose() + Q;

  //std::cout << "--- PREDICTION ---" << std::endl;
  //std::cout << "F: \n" << F << "\n" << std::endl;
  //std::cout << "Q: \n" << Q << "\n" << std::endl;
  //std::cout << "Estimation of P : \n" << P << "\n" << std::endl;
  return state_vector();
}

void PoseEstimator::update(const Eigen::VectorXd & measurements){
  // measurements is 'z' on the literature
  z = measurements;
  Eigen::MatrixXd h_ = h(x, dt);

  y = z - h_;
  H = jacobian_h(x, dt);
  R_update(dt);

  S = H * P * H.transpose() + R;
  K = P * H.transpose() * S.inverse();
  x = x + K * y;
  Eigen::MatrixXd Id = Eigen::MatrixXd::Identity(dim_state_vector, dim_state_vector);
  P = (Id - K * H) * P;

  std::cout << "--- PREDICTION ---" << std::endl;
  std::cout << "KALMAN GAIN: \n" << K << "\n" << std::endl;
  //std::cout << "Correction of State vector (x): \n" << x.transpose() << "\n" << std::endl;
  //std::cout << "Correction of P : \n" << P << "\n\n" << std::endl;
}

const Eigen::VectorXd PoseEstimator::state_vector(){
  return x;
}


Eigen::Vector3d PoseEstimator::get_position(){
  return Eigen::Vector3d(x(0,0), x(1,0), x(2,0));
}


void PoseEstimator::print_state_vector(){
  Eigen::Vector3d pos = x.block<3,1>(0,0);
  Eigen::Vector3d vel = x.block<3,1>(3,0);
  Eigen::Vector3d acc = x.block<3,1>(6,0);
  printf("\nEKF state: \ndt: %.4f seg.\n", dt);
  printf("Position:     [%.3f, %.3f, %.3f] (m)\n", pos.x(), pos.y(), pos.z());
  printf("Velocity:     [%.3f, %.3f, %.3f] (%.3f m/s)\n", vel.x(), vel.y(), vel.z(), vel.norm());
  printf("Acceleration: [%.3f, %.3f, %.3f] (%.3f m/s^2)\n\n", acc.x(), acc.y(), acc.z(), acc.norm());
}

Eigen::MatrixXd PoseEstimator::f(Eigen::MatrixXd & x, double dt){
  Eigen::MatrixXd pos = x.block<3,1>(0,0);
  Eigen::MatrixXd vel = x.block<3,1>(3,0);
  Eigen::MatrixXd acc = x.block<3,1>(6,0);

  x.block<3,1>(0,0) = pos + vel*dt; // + 0.5 * acc * dt*dt;
  x.block<3,1>(3,0) =       vel    +       acc * dt;
  x.block<3,1>(6,0) =                      acc;
  return x;
}

Eigen::MatrixXd PoseEstimator::jacobian_f(const Eigen::MatrixXd & x, double dt){
  /*
    |  1     dt    0  |
    |  0     1     dt |
    |  0     0     1  |
  */
  F = Eigen::MatrixXd::Identity(dim_state_vector, dim_state_vector);
  F.block<3,3>(0,3) = Eigen::MatrixXd::Identity(3,3) * dt;
  F.block<3,3>(3,6) = Eigen::MatrixXd::Identity(3,3) * dt;

  return F;
}

Eigen::MatrixXd PoseEstimator::h(const Eigen::MatrixXd & x, double dt){
  /* 
    h(x,k) = [ acc ] 
  */
  Eigen::MatrixXd h_(dim_measurements, 1);
  h_(0,0) = x(6,0);
  h_(1,0) = x(7,0);
  h_(2,0) = x(8,0);

  return h_;
}

Eigen::MatrixXd PoseEstimator::jacobian_h(Eigen::MatrixXd & x, double dt){
  /* 
    H = [ 0, 0, 1 ] 
  */
  Eigen::MatrixXd jH = Eigen::MatrixXd::Zero(dim_measurements, dim_state_vector);
  jH.block<3,3>(0,6) = Eigen::MatrixXd::Identity(dim_measurements, dim_measurements);
  return jH;
}

  
Eigen::MatrixXd PoseEstimator::R_update(double time) {
  R.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * SIGMA_ACC_MEASURE * time * time;
  return R;
}