
#include "AccIntegrator.h"

AccIntegrator::AccIntegrator(bool acc_due_gravity):
  _acc_due_gravity(acc_due_gravity)
{
  position.setZero();
  velocity.setZero();
  _gravity = GRAVITY;
}


Vector3d AccIntegrator::update(const Vector3d & acceleration, const Quaterniond & sensor_attitude, const double & dt)
{
  Vector3d acc(acceleration);
  if (_acc_due_gravity){
    acc = remove_gravity(acc, sensor_attitude);
    acc = sensor_attitude * acc; 
  }
  
  velocity = velocity + acc * dt;
  position = position + velocity * dt;
  return Vector3d(position);
}


Vector3d AccIntegrator::remove_gravity(const Vector3d & acceleration, const Quaterniond & sensor_attitude)
{
   // Normalization should not be necessary, because attitude must be normalizated always
  Quaterniond attitude_norm = sensor_attitude.normalized(); 

  Vector3d g_rot = attitude_norm.inverse() * _gravity;
  Vector3d linear_acc = acceleration - g_rot;
   
  return linear_acc;
}
