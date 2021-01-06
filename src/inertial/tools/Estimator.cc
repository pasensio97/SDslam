
#include "inertial/tools/Estimator.h"
#include <iostream>

double Estimator::scale(const Vector3d & start_A, const Vector3d & end_A, const Vector3d & start_B, const Vector3d & end_B)
{
  Vector3d delta_A = end_A - start_A;
  Vector3d delta_B = end_B - start_B;

  return Estimator::scale(delta_A, delta_B);
}

double Estimator::scale(const Vector3d & delta_A, const Vector3d & delta_B)
{

  double delta_A_norm = delta_A.norm();
  double delta_B_norm = delta_B.norm();

  if (delta_A_norm == 0 || delta_B_norm == 0)
    return 1.0;

  return delta_B_norm / delta_A_norm;   
}


Matrix3d Estimator::rotation(const Vector3d & start_A, const Vector3d & end_A, const Vector3d & start_B, const Vector3d & end_B)
{
  Vector3d delta_A = end_A - start_A;
  Vector3d delta_B = end_B - start_B;

  Vector3d delta_A_normalized = delta_A.normalized();
  Vector3d delta_B_normalized = delta_B.normalized();
  
  double rotation_angle = angle(delta_A_normalized, delta_B_normalized);
  Vector3d rotation_axis = delta_A_normalized.cross(delta_B_normalized);

  Matrix3d rot_A_to_B;
  rot_A_to_B = AngleAxisd(rotation_angle, rotation_axis.normalized());
  return rot_A_to_B;

  // As quaternion
  //Quaterniond rot; 
  //rot = AngleAxisd(rotation_angle, rotation_axis);
}

double Estimator::angle(const Vector3d & vector_A, const Vector3d & vector_B, bool in_degrees)
{
  Vector3d vector_A_normalized = vector_A.normalized();
  Vector3d vector_B_normalized = vector_B.normalized();
  
  double dot_product = vector_A_normalized.dot(vector_B_normalized);

  double rotation_angle = 0.0;
  if (dot_product > 1.0)
    rotation_angle = acos(1.0);
  else if (dot_product < -1.0)
    rotation_angle = acos(-1.0);
  else
    rotation_angle = acos(dot_product);

  if (in_degrees)
    return rotation_angle * (180 / M_PI);
  return rotation_angle;
  
}

Vector3d Estimator::correction_scale_factor(
    const Vector3d & start_imu, const Vector3d & end_imu,
    const Vector3d & start_vision, const Vector3d & end_vision, 
    const double & expected_scale)
{

  double imu_norm = (end_imu - start_imu).norm();
  Vector3d vision_normalized = (end_vision - start_vision).normalized();

  Vector3d new_end_vision = start_vision + vision_normalized * imu_norm * expected_scale;
  Vector3d mu(0,0,0);
  for (int i=0; i<3; i++){
    mu[i] = new_end_vision[i] / end_vision[i];
  }

  std::cout << "\t* Vision normalized: " << vision_normalized.transpose() << std::endl; 
  std::cout << "\t* IMU norm: " << imu_norm << std::endl; 
  std::cout << "\t* New v1: " << new_end_vision.transpose() << std::endl; 
  std::cout << "\t* mu: " << mu.transpose() << std::endl; 
  return mu;
}

double Estimator::mean(std::vector<double> vec){
  if (vec.empty()){
    return 0.0;
  }

  double mean_value = 0;
  int n = vec.size(); 
  for (double value : vec){
    mean_value += value;
  }
  mean_value /= n; 
  return mean_value;
}

Quaterniond Estimator::quat_difference(const Quaterniond & q_start, const Quaterniond & q_end)
{
  // diff * q_start = q_end
  return q_end.normalized() * q_start.normalized().inverse();
}