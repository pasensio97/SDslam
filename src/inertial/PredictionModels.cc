
#include "inertial/PredictionModels.h"
#include "inertial/tools/Estimator.h"
#include "src/Converter.h"
#include <numeric>

new_IMU_model::new_IMU_model(const double & acc_lpf_gain, const bool &  remove_gravity, const double & mad_gain):
  _acc_lpf(3, acc_lpf_gain), _acc_due_grav(remove_gravity), _att_estimator(mad_gain)
{
  _gravity = Vector3d(0.0, 0.0, 9.86055);
  _R_imu_to_world << 0,-1,0, 0,0,-1, 1,0,0;  // NWU as IMU default
  _scale = 1.0;

  _position.setZero();
  _velocity.setZero();

  _last_position.setZero();
  _last_velocity.setZero();
  _last_acceleration.setZero();
  _acc_init = false;

  pose_cam = Matrix4d::Identity();
  pose_world = Matrix4d::Identity();
  pose_imu = Matrix4d::Identity();
}

void new_IMU_model::reset(){
  _scale = 1;
  _position.setZero();
  _delta_pos.setZero();
  _velocity.setZero();
  _att_estimator.set_orientation(Quaterniond(1,0,0,0));

  _last_position.setZero();
  _last_velocity.setZero();
  _last_acceleration.setZero();
  _acc_init = false;

  _scale_buffer = std::vector<double>();

  pose_cam = Matrix4d::Identity();
  pose_world = Matrix4d::Identity();
  pose_imu = Matrix4d::Identity();
}

Vector3d new_IMU_model::_remove_gravity_test_2(const Vector3d & acc, const Quaterniond & attitude){
  Quaterniond attitude_norm = attitude.normalized();

  Vector3d g_rot = attitude_norm.inverse() * _gravity;
  Vector3d linear_acc = acc - g_rot;
   
  return linear_acc;
}


Vector3d new_IMU_model::_remove_gravity_test(const Vector3d & acc, const Quaterniond & attitude){
  Quaterniond attitude_norm = attitude.normalized();
  Vector3d linear_acc = attitude_norm * (acc - _gravity);

  return linear_acc;
}

Vector3d new_IMU_model::_remove_gravity(const Vector3d & acc, const Quaterniond & attitude){
  Quaterniond attitude_norm = attitude.normalized();

  Vector3d linear_acc = attitude_norm*(acc - _gravity);
  // 1- Rotate gravity from the Earth frame to the sensor frame
  //Vector3d g_rot = attitude_norm.inverse() * _gravity;  // inverse or not???
  // 2- Remove gravity from acceleration measurement
  //Vector3d linear_acc = acc - g_rot;
  // 3- Rotate linear acceleration to model frame
  //linear_acc = attitude_norm * linear_acc;
  std::cout << "Linear acc: " << linear_acc.transpose() << std::endl;
  return linear_acc;
}

Matrix4d new_IMU_model::predict(IMU_Measurements & imu, double & dt){
  _last_position = Vector3d(_position);

  _att_estimator.update(imu.acceleration(), imu.angular_velocity(), dt);

  Vector3d acc = imu.acceleration();
  if (_acc_due_grav){
    acc = _remove_gravity(acc, _att_estimator.get_orientation());
  }

  // _position = _position + _velocity * dt + 0.5 * acc * (dt*dt);  // Test with velocity instead _velocity
  // _velocity = _velocity + acc * dt;
  _position = _position + _velocity * dt + 0.5 * acc * (dt*dt);  // Test with velocity instead _velocity
  _velocity = _velocity + acc * dt;
  

  _delta_pos = _position - _last_position;
  
  _update_poses();
  
  return pose_imu;
}

Matrix4d new_IMU_model::get_world_pose(const Matrix4d & last_world_pose){
  Vector3d delta_pos_world = _R_imu_to_world* (_scale*_delta_pos);
  Vector3d pos_world = last_world_pose.block<3,1>(0,3) + delta_pos_world;
  Matrix3d att_world = _att_estimator.get_local_orientation().toRotationMatrix().inverse();
  return Converter::toSE3(att_world, pos_world);
}

void new_IMU_model::_update_poses(){
  // Pose imu
  pose_imu = Converter::toSE3(_att_estimator.get_orientation().toRotationMatrix(), _position);

  // Pose world
  Vector3d pos_w = _R_imu_to_world * (_last_position +  _scale*_delta_pos); // Pose IMU to world
  Matrix3d att_c = _att_estimator.get_local_orientation().toRotationMatrix();
  pose_world = Converter::toSE3(att_c.inverse(), pos_w);
  std::cout << "[INFO] * Scale:   " << _scale << std::endl;
  std::cout << "[INFO] * Delta pos imu:   " << _delta_pos.transpose()  << std::endl;
  std::cout << "[INFO] * Delta pos world: " << (_R_imu_to_world * (_scale*_delta_pos)).transpose()  << std::endl;
  std::cout << "[INFO] * Pose on imu frame:   " << _position.transpose() << std::endl;
  std::cout << "[INFO] * Pose on world frame: " << pos_w.transpose() << std::endl;
  // Pose cam
  pose_cam = Converter::toSE3(att_c, -(att_c * pos_w));
}

double new_IMU_model::estimate_scale(const Frame & curr_frame, const Frame & last_frame, bool add_to_buffer){
  Vector3d curr_frame_wpos = curr_frame.GetPoseInverse().block<3,1>(0,3);
  Vector3d last_frame_wpos = last_frame.GetPoseInverse().block<3,1>(0,3);
  Vector3d delta_pos_world  = curr_frame_wpos - last_frame_wpos;

  double scale = Estimator::scale(_delta_pos, delta_pos_world);
  std::cout << "[TEST] Estimate scale by motion model: " << scale << std::endl;
  if (add_to_buffer){
    _scale_buffer.push_back(scale);
  }
  return scale;
}

double new_IMU_model::estimate_scale(const Vector3d delta_pos_slam, bool add_to_buffer){
  double scale = Estimator::scale(_delta_pos, delta_pos_slam);
  std::cout << "[TEST] Estimate scale by motion model: " << scale << std::endl;
  if (add_to_buffer){
    _scale_buffer.push_back(scale);
  }
  return scale;
}

void new_IMU_model::correct_pose(const Frame & curr_frame, const Frame & last_frame, double dt, bool estimate_and_save_scale){
  Vector3d curr_frame_wpos = curr_frame.GetPoseInverse().block<3,1>(0,3);
  Vector3d last_frame_wpos = last_frame.GetPoseInverse().block<3,1>(0,3);
  Vector3d delta_pos_world  = curr_frame_wpos - last_frame_wpos;
  
  if (estimate_and_save_scale){
    estimate_scale(delta_pos_world);
  }

  _att_estimator.set_orientation_from_frame(curr_frame.GetPose());

  _delta_pos = _R_imu_to_world.transpose() * ( (1.0/_scale)*delta_pos_world );

  _position = _last_position + _delta_pos;
  _velocity = (_position - _last_position) / dt;
  std::cout << "scale on correction: " << _scale << std::endl;

  _update_poses();
}

void new_IMU_model::initialize(const Matrix4d & curr_world_pose, 
                               const Matrix4d & last_world_pose, 
                               double dt)
{
  reset();
  Vector3d curr_wpos = curr_world_pose.block<3,1>(0,3);
  Vector3d last_wpos = last_world_pose.block<3,1>(0,3);
  
  _last_position = _R_imu_to_world.transpose() * last_wpos;
  _position      = _R_imu_to_world.transpose() * curr_wpos;
  _velocity      = (_position - _last_position) / dt;

  _att_estimator.set_orientation_from_frame(Converter::inverted_pose(curr_world_pose));
}

double new_IMU_model::scale_buffer_mean(){
  if (_scale_buffer.empty()){
    std::cout << "[TEST] ESTO NO DEBERIA DE OCURRIR" << std::endl;
    return _scale; 
  }

  int n = _scale_buffer.size(); 
  double mean = 0.0;
  for (double scale_value : _scale_buffer){
    mean += scale_value;
  }
  mean /= n; 
  return mean;
}

void new_IMU_model::scale_buffer_clear(bool full){
  if (!_scale_buffer.empty()){
    double last_scale = _scale_buffer.back();
    _scale_buffer.clear();
    if (! full){
      _scale_buffer.push_back(last_scale);
    }
  }
}
