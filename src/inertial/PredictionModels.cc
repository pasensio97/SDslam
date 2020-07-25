
#include "inertial/PredictionModels.h"
#include "inertial/tools/Estimator.h"
#include "src/Converter.h"
#include <numeric>

// GT_PredictionModel

Matrix4d GT_PredictionModel::predict(Frame & curr_frame, 
                                     Frame & last_frame)
{
  Matrix4d new_pose = Matrix4d::Identity();

  // Attitude
  Quaterniond curr_R_gps(curr_frame.R_gps());
  Quaterniond last_R_gps(last_frame.R_gps());

  Quaterniond diff_R_gps = (curr_R_gps * last_R_gps.inverse()).normalized();
  Quaterniond new_rot = diff_R_gps * Quaterniond(last_frame.GetRotation()).normalized();
  new_pose.block<3,3>(0,0) = new_rot.normalized().toRotationMatrix();
  
  // Position
  Vector3d diff_t_gps = curr_frame.t_gps() - last_frame.t_gps();
  Vector3d translationSLAM = rotation_gps_to_slam * diff_t_gps;    
  Vector3d new_t = diff_R_gps * last_frame.GetPosition() + curr_R_gps.inverse() * translationSLAM;   

  new_pose.block<3,1>(0,3) = new_t;
  return new_pose;
}

void GT_PredictionModel::estimate_rotation_gps_to_slam(Frame & first_frame, 
                                                       Frame & curr_frame)
{
}


// --- IMU MODEL ---
IMU_model::IMU_model(const double & mad_gain):
  _pos_estimator(false),
  _att_estimator(mad_gain),
  _R_imu2w(Matrix3d::Identity()), 
  acc_lpf(LowPassFilter(3, 0.2)),
  ratio_lpf(LowPassFilter(1, 0.1)),
  it(0),
  _ratio(1),
  _remove_gravity(false)
{
  _pos_estimator.set_gravity(Vector3d(0.0, 0.0, 9.80655));

  // FOR TEST (use kitti matrix === Rot_nwu_to_cam)
  Matrix3d rotation_imu_to_velo;
  rotation_imu_to_velo << 9.999976e-01, 7.553071e-04, -2.035826e-03, 
                          -7.854027e-04, 9.998898e-01, -1.482298e-02,
                          2.024406e-03, 1.482454e-02,  9.998881e-01;
  Matrix3d rotation_velo_to_cam;
  rotation_velo_to_cam <<  7.967514e-03, -9.999679e-01, -8.462264e-04,
                          -2.771053e-03,  8.241710e-04, -9.999958e-01,
                            9.999644e-01,  7.969825e-03, -2.764397e-03;
  _R_imu2w = rotation_velo_to_cam * rotation_imu_to_velo;

  _last_vel.setZero();
  _linear_acc.setZero();
}

Matrix4d IMU_model::predict(IMU_Measurements & imu, double & dt, Frame & curr_frame, Frame & last_frame){
  // att
  _att_estimator.update(imu.acceleration(), imu.angular_velocity(), dt);
  Quaterniond att_cam = _att_estimator.get_local_orientation();  

  // pos
  Vector3d linear_acc;
  bool remove_grav = false;
  bool use_ratio = true;

  if(remove_grav){
    linear_acc = _pos_estimator.remove_gravity(imu.acceleration(), _att_estimator.get_orientation());
    linear_acc = (_R_imu2w * linear_acc); // acc on slam world
    linear_acc = acc_lpf.apply(linear_acc);
    _linear_acc = Vector3d(linear_acc);
    std::cout << "\tLINEAR ACCELERATION: " << linear_acc.transpose() << "- Norm:" << linear_acc.norm() << std::endl;
    //linear_acc /= 10;
  }
  else{
    linear_acc = imu.acceleration();
  }
  
  if (use_ratio && _ratio > 1e-10){
    //_ratio = 4.5; // 16.7
    linear_acc = linear_acc / (_ratio);
  }
  
  std::cout << "\tLINEAR ACCELERATION: " << linear_acc.transpose() << std::endl;
  Vector3d pos_world = _pos_estimator.update(linear_acc, dt); 
  Vector3d pos_cam = -(att_cam * pos_world);

  Matrix4d pose = Matrix4d::Identity();
  pose.block<3,3>(0,0) = att_cam.toRotationMatrix();
  pose.block<3,1>(0,3) = pos_cam;

  //test 
  position = pos_world;
  position_cam = pos_cam;
  attitude = _att_estimator.get_local_orientation();

  return pose;
}

Matrix4d IMU_model::predict(IMU_Measurements & imu, double & dt){
  // att
  _att_estimator.update(imu.acceleration(), imu.angular_velocity(), dt);
  Quaterniond att_cam = _att_estimator.get_local_orientation();  

  // pos
  Vector3d linear_acc;
  bool remove_grav = false;
  bool use_ratio = true;

  if(remove_grav){
    linear_acc = _pos_estimator.remove_gravity(imu.acceleration(), _att_estimator.get_orientation());
    linear_acc = (_R_imu2w * linear_acc); // acc on slam world
    linear_acc = acc_lpf.apply(linear_acc);
    _linear_acc = Vector3d(linear_acc);
    std::cout << "\tLINEAR ACCELERATION: " << linear_acc.transpose() << "- Norm:" << linear_acc.norm() << std::endl;
    //linear_acc /= 10;
  }
  else{
    linear_acc = imu.acceleration();
  }
  
  if (use_ratio && _ratio > 1e-10){
    //_ratio = 4.5; // 16.7
    linear_acc = linear_acc / (_ratio);
  }
  
  std::cout << "\tLINEAR ACCELERATION: " << linear_acc.transpose() << std::endl;
  Vector3d pos_world = _pos_estimator.update(linear_acc, dt); 
  Vector3d pos_cam = -(att_cam * pos_world);

  Matrix4d pose = Matrix4d::Identity();
  pose.block<3,3>(0,0) = att_cam.toRotationMatrix();
  pose.block<3,1>(0,3) = pos_cam;

  //test 
  position = pos_world;
  position_cam = pos_cam;
  attitude = _att_estimator.get_local_orientation();

  return pose;
}


void IMU_model::correct_pose(Frame & curr_frame, Frame & last_frame, double dt){
  int n = 1;
  if (it % n == 0){
    _att_estimator.set_orientation_from_frame(curr_frame.GetPose());
    Vector3d curr_pos = -curr_frame.GetRotation().transpose() * curr_frame.GetPosition(); 
    Vector3d last_pos = -last_frame.GetRotation().transpose() * last_frame.GetPosition(); 
    _pos_estimator.correct_pos_and_vel(curr_pos, last_pos, dt);
    
    // test
    Vector3d acc_corr =  (_pos_estimator.velocity() - _last_vel) / dt;
    std::cout << "\tACC correct: " << acc_corr.transpose() << "- Norm:" << acc_corr.norm() << std::endl;
    
    VectorXd ratio(1);
    ratio[0] = _linear_acc.norm() / acc_corr.norm();
    //_ratio = ratio_lpf.apply(ratio)[0];
    std::cout << "\tRatio: " << _ratio << std::endl;
    _last_vel = Vector3d(_pos_estimator.velocity());
  }
  it++;
}












// --- NEW IMU MODEL --
new_IMU_model::new_IMU_model(const double & acc_lpf_gain, const bool &  remove_gravity, const double & mad_gain):
  _acc_lpf(3, acc_lpf_gain), _acc_due_grav(remove_gravity), _att_estimator(mad_gain)
{
  _gravity = Vector3d(0.0, 0.0, 9.86055);
  _R_imu_to_world << 0,-1,0, 0,0,-1, 1,0,0;  // NWU as IMU default
  _scale = 1.0;

  _position.setZero();
  _last_position.setZero();
  _velocity.setZero();
  _last_velocity.setZero();
  pose_cam = Matrix4d::Identity();
  pose_world = Matrix4d::Identity();
  pose_imu = Matrix4d::Identity();
}

void new_IMU_model::reset(){
  _position.setZero();
  _velocity.setZero();
  _att_estimator.set_orientation(Quaterniond(1,0,0,0));
  
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

  // 1- Rotate gravity from the Earth frame to the sensor frame
  Vector3d g_rot = attitude_norm.inverse() * _gravity;  // inverse or not???
  // 2- Remove gravity from acceleration measurement
  Vector3d linear_acc = acc - g_rot;
  // 3- Rotate linear acceleration to model frame
  linear_acc = attitude_norm * linear_acc;

  return linear_acc;
}

Matrix4d new_IMU_model::predict(IMU_Measurements & imu, double & dt){
  _last_position = Vector3d(_position);

  _att_estimator.update(imu.acceleration(), imu.angular_velocity(), dt);

  Vector3d acc = imu.acceleration();
  if (_acc_due_grav){
    acc = _remove_gravity(acc, _att_estimator.get_orientation());
  }

  //acc *= _scale;
  _velocity = _velocity + acc * dt;
  _position = _position + _velocity * dt;
  
  _update_poses();
  
  return pose_cam;
}

void new_IMU_model::_update_poses(){
  // Pose imu
  pose_imu = Converter::toSE3(_att_estimator.get_orientation().toRotationMatrix(), _position);

  // Pose world
  Vector3d pos_w = _R_imu_to_world * (_position * _scale); // Pose IMU to world
  Matrix3d att_c = _att_estimator.get_local_orientation().toRotationMatrix();
  pose_world = Converter::toSE3(att_c.inverse(), pos_w);

  // Pose cam
  pose_cam = Converter::toSE3(att_c, -(att_c * pos_w));
}

double new_IMU_model::estimate_scale(Frame & curr_frame, Frame & last_frame, bool add_to_buffer){
  Vector3d curr_frame_wpos = curr_frame.GetPoseInverse().block<3,1>(0,3);
  Vector3d last_frame_wpos = last_frame.GetPoseInverse().block<3,1>(0,3);
  double scale = Estimator::scale(_R_imu_to_world * get_last_position_imu(),
                                  _R_imu_to_world * get_position_imu(),
                                  last_frame_wpos, 
                                  curr_frame_wpos);
  std::cout << "SLAM 0: " << last_frame_wpos.transpose() << std::endl;
  std::cout << "SLAM 1: " << curr_frame_wpos.transpose() << std::endl;
  std::cout << "IMU 0: " << (_R_imu_to_world * get_last_position_imu()).transpose() << std::endl;
  std::cout << "IMU 1: " << (_R_imu_to_world * get_position_imu()).transpose() << std::endl;
  std::cout << "[TEST] Estimate scale by motion model: " << scale << std::endl;
  if (add_to_buffer){
    _scale_buffer.push_back(scale);
  }
  return scale;
}

void new_IMU_model::correct_pose(Frame & curr_frame, Frame & last_frame, double dt){
  Vector3d curr_frame_wpos = curr_frame.GetPoseInverse().block<3,1>(0,3);
  Vector3d last_frame_wpos = last_frame.GetPoseInverse().block<3,1>(0,3);
  
  estimate_scale(curr_frame, last_frame);
  _att_estimator.set_orientation_from_frame(curr_frame.GetPose());

  Vector3d curr_pos_imu = _R_imu_to_world.transpose() * (curr_frame_wpos / _scale);
  Vector3d last_pos_imu = _R_imu_to_world.transpose() * (last_frame_wpos / _scale);

  // curr_pos_imu *= (1.0 / _scale);
  // last_pos_imu *= (1.0 / _scale);
  _position = curr_pos_imu;
  _velocity = (curr_pos_imu - last_pos_imu) / dt;
  std::cout << "IMU model vel: " << _velocity.transpose() << std::endl;

  _update_poses();
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

void new_IMU_model::scale_buffer_clear(){
  if (!_scale_buffer.empty()){
    double last_scale = _scale_buffer.back();
    _scale_buffer.clear();
    _scale_buffer.push_back(last_scale);
  }
}

/*

Matrix4d new_IMU_model::predict(IMU_Measurements & imu, double & dt){
  _att_estimator.update(imu.acceleration(), imu.angular_velocity(), dt);

  //Vector3d acc = _acc_lpf.apply(imu.acceleration());
  Vector3d acc = imu.acceleration();
  if (_acc_due_grav){
    acc = _remove_gravity(acc, _att_estimator.get_orientation());
     // _att_estimator.get_orientation() * (acc - g);
    acc = _att_estimator.get_orientation() * acc;
  }
  
  acc *= _scale;
  _velocity = _velocity + acc * dt;
  _position = _position + _velocity * dt;
  
  _update_poses();
  
  return pose_cam;
}

void new_IMU_model::_update_poses(){
  Vector3d pos_w = _R_imu_to_world * _position; // Pose IMU to world
  Matrix3d att_c = _att_estimator.get_local_orientation().toRotationMatrix();

  // Pose cam
  pose_cam = Matrix4d::Identity();
  pose_cam.block<3,3>(0,0) = att_c;
  pose_cam.block<3,1>(0,3) = -(att_c * pos_w);

  // Pose world
  pose_world = Matrix4d::Identity();
  pose_world.block<3,3>(0,0) = att_c.inverse();
  pose_world.block<3,1>(0,3) = pos_w;
  std::cout << "WORLD POSE new IMU MODEL: \n" << pose_world << "\n" << std::endl; 

  // Pose imu
  pose_imu = Matrix4d::Identity();
  pose_imu.block<3,3>(0,0) = _att_estimator.get_orientation().toRotationMatrix();
  pose_imu.block<3,1>(0,3) = _position;
}

void new_IMU_model::correct_pose(Frame & curr_frame, Frame & last_frame, double dt){

  _att_estimator.set_orientation_from_frame(curr_frame.GetPose());

  Vector3d curr_pos_imu = _R_imu_to_world.transpose() * curr_frame.GetPoseInverse().block<3,1>(0,3);
  Vector3d last_pos_imu = _R_imu_to_world.transpose() * last_frame.GetPoseInverse().block<3,1>(0,3);

  _position = curr_pos_imu;
  _velocity = (curr_pos_imu - last_pos_imu) / dt;

  _update_poses();
}
*/



/*
// WTF??? esto funciona!
Vector3d new_IMU_model::_remove_gravity(const Vector3d & acc, const Quaterniond & attitude){
  Quaterniond attitude_norm = attitude.normalized();

  Vector3d acc_l = acc - _gravity;
  Vector3d g_rot = attitude_norm * acc_l;
   
  return g_rot;
}
*/














// --- GPS ---
GPS_IMU_model::GPS_IMU_model(const double & mad_gain):
 _att_estimator(Madgwick(mad_gain)),
 _pose(Matrix4d::Identity()), _wpose(Matrix4d::Identity())
{}

Matrix4d GPS_IMU_model::estimate_pose(IMU_Measurements & imu, double & dt, Frame & curr_frame, Frame & last_frame, double scale){
  // Attitude with IMU
  _att_estimator.update(imu.acceleration(), imu.angular_velocity(), dt);
  Matrix3d R = _att_estimator.get_local_orientation().toRotationMatrix();  
  
  // Position with GPS
  Vector3d delta_t = (curr_frame.t_gps() - last_frame.t_gps()) / scale; // cambiar a multiplicacion
  Vector3d last_t = -last_frame.GetRotation().transpose() * last_frame.GetPosition(); 
  Vector3d tw = last_t + delta_t;
  Vector3d t = -(R * tw);

  // WPose
  _wpose = Matrix4d::Identity();
  _wpose.block<3,3>(0,0) = R.inverse();
  _wpose.block<3,1>(0,3) = tw;

  // Pose
  Matrix4d pose = Matrix4d::Identity();
  pose.block<3,3>(0,0) = R;
  pose.block<3,1>(0,3) = t;
  _pose = Matrix4d(pose);

  return pose;
}


