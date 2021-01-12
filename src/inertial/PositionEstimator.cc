#include "inertial/PositionEstimator.h"
#include <Eigen/Dense>



PositionEstimator::PositionEstimator(const bool & use_linear_acc):
  _lpf_acceleration(LowPassFilter(3, 0.3)),
  _gravity(Vector3d()), 
  _use_linear_acc(use_linear_acc),
  _initialized(false),
  _it(0)
{
  _last_acceleration.setZero();
  _acceleration.setZero();
  _velocity.setZero();
  _position.setZero();
}

PositionEstimator::PositionEstimator(const double & alpha_acceleration, 
                                     const Eigen::Vector3d & gravity_vector):
  _lpf_acceleration(LowPassFilter(3, alpha_acceleration)),
  _gravity(gravity_vector), 
  _use_linear_acc(false),
  _initialized(false),
  _it(0)
{
  _last_acceleration.setZero();
  _acceleration.setZero();
  _velocity.setZero();
  _position.setZero();
}


Eigen::Vector3d PositionEstimator::remove_gravity(const Eigen::Vector3d & acceleration, 
                                                  const Eigen::Quaterniond & sensor_attitude,
                                                  const bool & verbose)
{
  Quaterniond attitude_norm = sensor_attitude.normalized();

  Vector3d g_rot = attitude_norm.inverse() * _gravity;
  Vector3d linear_acc = acceleration - g_rot;

  if (verbose){
    auto euler = sensor_attitude.toRotationMatrix().eulerAngles(0, 1, 2);
    printf("\nRemove gravity force: \n");
    printf("\tSensor attitude: (x: %.3f, y: %.3f, z: %.3f, w: %.3f)\n", sensor_attitude.x(), sensor_attitude.y(), sensor_attitude.z(), sensor_attitude.w());
    printf("\tEuler angles:    [%.3fº, %.3f, %.3fº]\n" , euler[0], euler[1], euler[2]);
    printf("\tSensor measure:  (x: %.3f, y: %.3f, z: %.3f)\n", acceleration.x(), acceleration.y(), acceleration.z());
    printf("\tGravity rotated: (x: %.3f, y: %.3f, z: %.3f)\n", g_rot.x(), g_rot.y(), g_rot.z());
    printf("\tLinear acc:      (x: %.3f, y: %.3f, z: %.3f)\n", linear_acc.x(), linear_acc.y(), linear_acc.z());
  }

   
  return linear_acc;
}


Eigen::Vector3d PositionEstimator::update(const Eigen::Vector3d & acceleration, 
                                          const Eigen::Quaterniond & sensor_attitude, 
                                          const double & dt,
                                          const bool & verbose)
{

  Vector3d filt_acc = _lpf_acceleration.apply(acceleration);
  Vector3d linear_acc = remove_gravity(filt_acc, sensor_attitude, verbose);

  
  _acceleration = linear_acc; 

  // integrate acceleration to obtein velocity and position
  _velocity = _velocity + _acceleration * dt;
  _position = _position + _velocity * dt;

  if (verbose){
    printf("\nPosition state: \n\tdt: %.4f seg.\n", dt);
    printf("\tAcceleration: [%.3f, %.3f, %.3f] (%.3f m/s^2)\n", _acceleration.x(), _acceleration.y(), _acceleration.z(), _acceleration.norm());
    printf("\tVelocity:     [%.3f, %.3f, %.3f] (%.3f m/s)\n", _velocity.x(), _velocity.y(), _velocity.z(), _velocity.norm());
    printf("\tPosition:     [%.3f, %.3f, %.3f] (m)\n\n", _position.x(), _position.y(), _position.z());
  }

  _it++;
  return position();

}



Eigen::Vector3d PositionEstimator::update(const Eigen::Vector3d & linear_acceleration, 
                                          const double & dt,
                                          const bool & verbose)
{
  _acceleration = linear_acceleration; 
  // integrate acceleration to obtein velocity and position
  _velocity = _velocity + _acceleration * dt;
  _position = _position + _velocity * dt;

  return position();
}

void PositionEstimator::correct_pos_and_vel(const Vector3d & curr_pos, const Vector3d & last_pose, double dt){
  _position = curr_pos;
  _velocity = (curr_pos - last_pose) / dt;
  //_acceleration = (_velocity - _last_velocity) / dt;
  //_last_velocity = _velocity;
}