
#include "inertial/tools/Estimator.h"
#include <iostream>

double scale(const Vector3d & start_A, const Vector3d & end_A, const Vector3d & start_B, const Vector3d & end_B)
{
  Vector3d delta_A = end_A - start_A;
  Vector3d delta_B = end_B - start_B;

  double delta_A_norm = delta_A.norm();
  double delta_B_norm = delta_B.norm();

  if (delta_A_norm == 0 || delta_B_norm == 0)
    return 1.0;

  return delta_B_norm / delta_A_norm;   
}

double scale(const Vector3d & delta_A, const Vector3d & delta_B)
{

  double delta_A_norm = delta_A.norm();
  double delta_B_norm = delta_B.norm();

  if (delta_A_norm == 0 || delta_B_norm == 0)
    return 1.0;

  return delta_B_norm / delta_A_norm;   
}


Matrix3d rotation(const Vector3d & start_A, const Vector3d & end_A, const Vector3d & start_B, const Vector3d & end_B)
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

double angle(const Vector3d & vector_A, const Vector3d & vector_B, bool in_degrees)
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

double correction_scale_factor(const Vector3d & delta_imu, const Vector3d & delta_vision, const double & expected_scale)
{
  
  double delta_imu_norm = delta_imu.norm();
  double delta_vis_norm = delta_vision.norm();

  if (delta_imu_norm == 0 || delta_vis_norm == 0)
    return 0.0; // that should not happen
  
  
  return (delta_imu_norm * expected_scale) / delta_vis_norm;   
}