#define _USE_MATH_DEFINES

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include "System.h"
#include "Tracking.h"
#include "Map.h"
#include "Config.h"
#include "ui/Viewer.h"
#include "ui/FrameDrawer.h"
#include "ui/MapDrawer.h"
#include "debug_tools/seq_dumper.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "sensors/EKF.h"
#include "sensors/IMU.h"
#include <math.h>
#include "opencv2/opencv.hpp"
#include "geometry_msgs/PoseStamped.h"

#include "inertial/IMU_Measurements.h"
#include "inertial/Madgwick.h"
#include "inertial/Madgwick_ros.h"
#include "inertial/PoseEstimator.h"
#include "inertial/PositionEstimator.h"
#include "inertial/tools/filters.h"

using namespace std;

class IMU_Reader {
 public:
  IMU_Reader() {
    updated_ = false;
    
  }

  void read_imu_msg(const sensor_msgs::ImuConstPtr& msgIMU) {
    updated_ = true;

    double timestamp = msgIMU->header.stamp.toSec();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d(msgIMU->angular_velocity.x, //x
                                                       msgIMU->angular_velocity.y, //z
                                                       msgIMU->angular_velocity.z);//y
    Eigen::Vector3d acceleration = Eigen::Vector3d(msgIMU->linear_acceleration.x,
                                                   msgIMU->linear_acceleration.y, 
                                                   msgIMU->linear_acceleration.z);   
    ros_time  = msgIMU->header.stamp;
    imu_data_ = IMU_Measurements(timestamp, acceleration, angular_velocity);
  }

  void read_acc_and_gyro_msg(const sensor_msgs::ImuConstPtr& msg_acc, const sensor_msgs::ImuConstPtr& msg_gyro) {
    updated_ = true;

    double acc_t =  msg_acc->header.stamp.toSec();
    double gyro_t =  msg_gyro->header.stamp.toSec();
    double timestamp = (acc_t > gyro_t) ? acc_t : gyro_t;

    Eigen::Vector3d acceleration = Eigen::Vector3d(msg_acc->linear_acceleration.x,
                                                   msg_acc->linear_acceleration.y, 
                                                   msg_acc->linear_acceleration.z);   
    Eigen::Vector3d angular_velocity = Eigen::Vector3d(msg_gyro->angular_velocity.x,
                                                       msg_gyro->angular_velocity.y, 
                                                       msg_gyro->angular_velocity.z);                                               
    ros_time  = msg_acc->header.stamp;
    imu_data_ = IMU_Measurements(timestamp, acceleration, angular_velocity);
  }

  IMU_Measurements get_data() {
    updated_ = false;
    return imu_data_;
  }

  ros::Time get_ros_time() {return ros_time;}

  bool has_new_data() {
    return updated_;
  }

 private:
  bool updated_;
  IMU_Measurements imu_data_;
  ros::Time ros_time;
};

class TF_Publisher {
 private:
  tf::TransformBroadcaster _br;

 public:
  const std::string base_frame;
  const std::string child_frame;


  TF_Publisher(ros::NodeHandle n, const string base_frame, const string child_frame):
     base_frame(base_frame), child_frame(child_frame)
  {
  }

  void publish(Vector3d position, Quaterniond orientation){

    ros::Time time_now = ros::Time::now();

    tf::Vector3 tf_position(position.x(), position.y(), position.z());
    tf::Quaternion tf_orientation(orientation.x(), orientation.y(), orientation.z(), orientation.w()); 

    tf::Transform tf;
    tf.setOrigin(tf_position);  
    tf.setRotation(tf_orientation);

    tf::StampedTransform stamped_transform = tf::StampedTransform(tf, time_now, base_frame, child_frame);
    _br.sendTransform(stamped_transform);
  }
};

Vector3d custom_remove_gravity(const Vector3d & acc, const Quaterniond & attitude, const Vector3d & g);

class Odometry{
 private:
  Vector3d _acc;
  Vector3d _last_acc;
  Vector3d _pose;
  Vector3d _velocity;
  double _last_t;
  bool _initialize;
  double _global_t;
  Vector3d _g;

  double _th;
  LowPassFilter lpf_v = LowPassFilter(3, 0.25);

 public:
  Odometry(double th){
    _acc.setZero();
    _pose.setZero();
    _velocity.setZero();
    _last_t = 0.0;
    _initialize = false;
    _global_t = 0.0;

    _th = th;

    printf("Este metodo requiere de la aceleracion LINEAL\n");
  }

  void update(const Vector3d & linear_acc, const Quaterniond & orientation, double dt){
    _velocity = _velocity + linear_acc * dt;
    //_velocity = lpf_v.apply(_velocity, 0.2);

    _pose = _pose + _velocity * dt;
    
    printf("\nIntegration state: \ndt: %.4f seg.\n", dt);
    printf("Position:     [%.3f, %.3f, %.3f] (m)\n", _pose.x(), _pose.y(), _pose.z());
    printf("Velocity:     [%.3f, %.3f, %.3f] (%.3f m/s)\n", _velocity.x(), _velocity.y(), _velocity.z(), _velocity.norm());
    printf("Acceleration: [%.3f, %.3f, %.3f] (%.3f m/s^2)\n\n", linear_acc.x(), linear_acc.y(), linear_acc.z(), linear_acc.norm());
  }

  void update(const Vector3d & acc, const Quaterniond & orientation, double dt, double th){
    // Non zero update
    Vector3d linear_acc;

    for(int i=0; i<3; i++){
      linear_acc[i] = (abs(acc[i]) < th) ? 0.0 : acc[i];
    }

    _velocity = _velocity + linear_acc * dt;
    printf("PRE LPF Velocity:     [%.3f, %.3f, %.3f] (%.3f m/s)\n", _velocity.x(), _velocity.y(), _velocity.z(), _velocity.norm());
    _velocity = lpf_v.apply(_velocity);
    printf("Post LPF Velocity:    [%.3f, %.3f, %.3f] (%.3f m/s)\n", _velocity.x(), _velocity.y(), _velocity.z(), _velocity.norm());
    _pose = _pose + _velocity * dt;

    printf("\nIntegration state: \ndt: %.4f seg.\n", dt);
    printf("Position:     [%.3f, %.3f, %.3f] (m)\n", _pose.x(), _pose.y(), _pose.z());
    printf("Velocity:     [%.3f, %.3f, %.3f] (%.3f m/s)\n", _velocity.x(), _velocity.y(), _velocity.z(), _velocity.norm());
    printf("Acceleration: [%.3f, %.3f, %.3f] (%.3f m/s^2)\n\n", linear_acc.x(), linear_acc.y(), linear_acc.z(), linear_acc.norm());
  }

void update_2(const Vector3d & linear_acc, const Quaterniond & orientation, double dt){
    _acc = (_acc + linear_acc) / 2;
    _velocity = _velocity + _acc * dt;
    //_velocity = lpf_v.apply(_velocity, 0.2);

    _pose = _pose + _velocity * dt;
    
    printf("\nIntegration state: \ndt: %.4f seg.\n", dt);
    printf("Position:     [%.3f, %.3f, %.3f] (m)\n", _pose.x(), _pose.y(), _pose.z());
    printf("Velocity:     [%.3f, %.3f, %.3f] (%.3f m/s)\n", _velocity.x(), _velocity.y(), _velocity.z(), _velocity.norm());
    printf("Acceleration: [%.3f, %.3f, %.3f] (%.3f m/s^2)\n\n", _acc.x(), _acc.y(), _acc.z(), _acc.norm());
  }

  void update_3(const Vector3d & raw_linear_acc, const Quaterniond & orientation, double dt){
    Vector3d linear_acc = raw_linear_acc - _last_acc;
    _last_acc = raw_linear_acc;

    _velocity = _velocity + linear_acc * dt;
    _pose = _pose + _velocity * dt;

    printf("\nIntegration state: \ndt: %.4f seg.\n", dt);
    printf("Position:     [%.3f, %.3f, %.3f] (m)\n", _pose.x(), _pose.y(), _pose.z());
    printf("Velocity:     [%.3f, %.3f, %.3f] (%.3f m/s)\n", _velocity.x(), _velocity.y(), _velocity.z(), _velocity.norm());
    printf("Acceleration: [%.3f, %.3f, %.3f] (%.3f m/s^2)\n\n", linear_acc.x(), linear_acc.y(), linear_acc.z(), linear_acc.norm());
  }

  Vector3d pose(){
    return _pose;
  }
};


/*
  Orientation must stay in NWU coordinate system
*/
Vector3d remove_gravity_base(Vector3d acc, Quaterniond  orientation, string cs = "NWU"){  
  Vector3d g(0, 0, 9.80665);
  if (cs == "ENU"){
    cout << "Transform acceleration from ENU to NWU to remove gravity." << endl;
    acc = Vector3d(-acc.y(), -acc.x(), acc.z());  // ENU to NWU
  }
  else if (cs == "WUN"){  // d435i
    cout << "Transform acceleration from ENU to NWU to remove gravity." << endl;
    acc = Vector3d(-acc.y(), -acc.x(), acc.z());  // ENU to NWU
  }

  orientation.normalize();

  Vector3d g_rot = orientation.toRotationMatrix().inverse() * g;
  Vector3d linear_acc;
  linear_acc =  acc - g_rot;

  printf("Sensor acc:  (x: %.3f, y: %.3f, z: %.3f)\n", acc.x(), acc.y(), acc.z());
  printf("G direction: (x: %.3f, y: %.3f, z: %.3f)\n", g_rot.x(), g_rot.y(), g_rot.z());
  printf("Lin. acc:    (x: %.3f, y: %.3f, z: %.3f)\n\n", linear_acc.x(), linear_acc.y(), linear_acc.z());

  return linear_acc;
}


/*
  Orientation and acceleration must be in the same coordinate system.
  * The orientation generated by madgwick is in the NWU
*/
Vector3d remove_gravity(Vector3d acc, Quaterniond  orientation, bool verbose=false){  

  orientation.normalize();
  Vector3d g(0, 0, 10.12/*9.80665*/);

  Vector3d g_rot = orientation.inverse() * g;
  Vector3d linear_acc = acc - g_rot;

  if (verbose){
    printf("Sensor measure:  (x: %.3f, y: %.3f, z: %.3f)\n", acc.x(), acc.y(), acc.z());
    printf("Gravity rotate:  (x: %.3f, y: %.3f, z: %.3f)\n", g_rot.x(), g_rot.y(), g_rot.z());
    printf("Linear acceler:  (x: %.3f, y: %.3f, z: %.3f)\n", linear_acc.x(), linear_acc.y(), linear_acc.z());
  }
  return linear_acc;
}



class Vector_Publisher{
 private:
    string frame_id;
    ros::Publisher pub;

 public:
  Vector_Publisher(ros::NodeHandle nodehandle, const string &frame, const string &topic){
    pub = nodehandle.advertise<geometry_msgs::PoseStamped>(topic, 100);
    frame_id = frame;
  }

  void publish(const Vector3d & position, const Vector3d & orientation){

    tf::Vector3 tf_position(position.x(), position.y(), position.z());
    tf::Quaternion tf_orientation(tf::Vector3(orientation.x(), orientation.y(), orientation.z()), 1);

    tf::Transform new_transform;
    new_transform.setOrigin(tf_position);
    new_transform.setRotation(tf_orientation);

    geometry_msgs::PoseStamped pose_msgs;
    pose_msgs.header.stamp = ros::Time::now();
    pose_msgs.header.frame_id = frame_id;
    tf::poseTFToMsg(new_transform, pose_msgs.pose);
    pub.publish(pose_msgs);
  }
};

class Imu_Publisher{
 private:
    string frame_id;
    ros::Publisher pub;
    int seq;
 public:
  Imu_Publisher(ros::NodeHandle nodehandle, const string &frame, const string &topic){
    pub = nodehandle.advertise<sensor_msgs::Imu>(topic, 100);
    frame_id = frame;
    seq = 0;
  }

  void publish(const Vector3d & acc, const Vector3d & gyr, ros::Time timestamp = ros::Time::now()){
    sensor_msgs::Imu imu_msg;
    
    imu_msg.header.stamp = timestamp;
    imu_msg.header.frame_id = frame_id;
    imu_msg.header.seq = seq;
    seq++;

    imu_msg.orientation_covariance[0] = -1;

    imu_msg.linear_acceleration.x = acc.x();
    imu_msg.linear_acceleration.y = acc.y();
    imu_msg.linear_acceleration.z = acc.z();
    imu_msg.linear_acceleration_covariance[0] = -1;

    imu_msg.angular_velocity.x = gyr.x();
    imu_msg.angular_velocity.y = gyr.y();
    imu_msg.angular_velocity.z = gyr.z();
    imu_msg.angular_velocity_covariance[0] = -1;

    pub.publish(imu_msg);

    seq++;
  }
};

Vector3d nwu_to_enu(const Vector3d & v){
  return Vector3d(-v.y(), -v.x(), v.z());
}

void update_gravity(Eigen::Vector3d &gravity, const Eigen::Vector3d &acc, double time) {
  // Low pass filter
  double alpha =  0.27 / (0.27 + time);

  gravity(0) = alpha * gravity(0) + (1 - alpha) * acc(0);
  gravity(1) = alpha * gravity(1) + (1 - alpha) * acc(1);
  gravity(2) = alpha * gravity(2) + (1 - alpha) * acc(2);
}

// Main usando la informacion del la imu de la camara D435i para predecir su orientacion
int main_d435i(int argc, char **argv){
  ros::init(argc, argv, "IMU_TEST");
  ros::start();

  ros::NodeHandle n;

  // Readers
  IMU_Reader reader_arduino;
  IMU_Reader reader_intel;

  // Subscribe to topics
  ros::Subscriber imu_sub = n.subscribe("/imu_101", 100, &IMU_Reader::read_imu_msg, &reader_arduino);

  message_filters::Subscriber<sensor_msgs::Imu> acc_sun(n, "/camera/accel/sample", 100);
  message_filters::Subscriber<sensor_msgs::Imu> gyro_sub(n, "/camera/gyro/sample", 100);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(100), acc_sun, gyro_sub);
  sync.registerCallback(boost::bind(&IMU_Reader::read_acc_and_gyro_msg, &reader_intel, _1, _2));

  // TF Publisher
  TF_Publisher tf_pub_arduino_orientation = TF_Publisher(n, "odom", "arduimu_orientation");
  TF_Publisher tf_pub_intel_orientation =   TF_Publisher(n, "odom", "intel_orientation");
  TF_Publisher tf_pub_arduino = TF_Publisher(n, "odom", "arduimu_pose");
  TF_Publisher tf_pub_intel =   TF_Publisher(n, "odom", "intel_pose");

  // Acc publisher
  Imu_Publisher imu_pub_arduino_acc = Imu_Publisher(n, "arduimu_orientation", "arduimu_acc");
  Imu_Publisher imu_pub_intel_acc =   Imu_Publisher(n, "intel_orientation", "intel_acc");

  // IMU orientation calculator
  Madgwick madgwick_arduino = Madgwick(0.01); 
  Madgwick madgwick_intel =   Madgwick(0.01); 


  // IMU pose estimator
  PoseEstimator pose_estimator_arduino = PoseEstimator();
  PoseEstimator pose_estimator_intel   = PoseEstimator();

  // AUX
  Vector3d static_pose_arduino(0, -0.5, 0);
  Vector3d static_pose_intel  (0,  0.5, 0);

  Odometry pose_integrate = Odometry(0.5);
  TF_Publisher tf_pub_intel_integration =   TF_Publisher(n, "odom", "pose_integral_INTEL");


  // TEST ODOMETRIA
  Madgwick attitude_estimator_intel =   Madgwick(0.01); 
  Vector3d g(0.0, 0.0, 10.0);
  PositionEstimator position_estimator_intel = PositionEstimator(0.3, g);

  cout << "Imu node initialized correctly for D435i dataset." << std::endl;

  int FPS = 30;
  double last_t_arduimu=0.0;
  double last_t_intel=0.0;


  int intel_count = 0;
  Vector3d simple_acc  (0,0,0);
  PoseEstimator pose_estimator_intel_test = PoseEstimator();
  TF_Publisher tf_pub_intel_test =   TF_Publisher(n, "odom", "intel_pose_test");
  Imu_Publisher imu_pub_intel_acc_test =   Imu_Publisher(n, "intel_orientation", "intel_acc_test");

  LowPassFilter lpf = LowPassFilter(3, 0.2);

  Quaterniond attitude;
  Vector3d position;

  bool use_arduino = false;
  bool debug_intel = true;
  
  // intel debug vars
  Imu_Publisher pub_debug_linear_acc = Imu_Publisher(n, "odom", "debug_intel_linear_acc");
  

  ros::Rate r(30);
  while (ros::ok()) {
    // Arduimu (ENU)
    if (use_arduino && reader_arduino.has_new_data()){
      IMU_Measurements imu_data = reader_arduino.get_data();

      double t = imu_data.timestamp();
      double dt = t - last_t_arduimu;
      last_t_arduimu = t;

      if (dt == t) // first iteration
        dt = 1 / FPS; 

      // Update orientation with measurements
      madgwick_arduino.update(imu_data.acceleration(), imu_data.angular_velocity(), dt);

      Vector3d grav_acc = imu_data.acceleration();
      Vector3d linear_acc = remove_gravity(grav_acc, madgwick_arduino.get_orientation());
      
      // Pose estimator - EKF
      pose_estimator_arduino.predict(dt);
      Vector3d pose_arduino = pose_estimator_arduino.get_position();
      pose_estimator_arduino.update(linear_acc);

      // Publish pose (EKF + Madgwick)
      tf_pub_arduino.publish(pose_arduino, madgwick_arduino.get_orientation());

      // Publish orientation (Madgwick)
      tf_pub_arduino_orientation.publish(static_pose_arduino, madgwick_arduino.get_orientation());

      // Publish linear acceleration vector (linear acceleration)
      imu_pub_arduino_acc.publish(linear_acc, imu_data.angular_velocity());
    }

    // D435i (NWU)
    if (reader_intel.has_new_data()) {
      IMU_Measurements imu_data = reader_intel.get_data();

      double t = imu_data.timestamp();
      double dt = t - last_t_intel;
      last_t_intel = t;

      if (dt == t) // first iteration
        continue;

      // Rotate IMU data from NWU to ENU
      Vector3d acc = imu_data.acceleration();
      Vector3d gyr = imu_data.angular_velocity();
      Vector3d enu_acc(-acc.y(), acc.x(), acc.z());
      Vector3d enu_gyr(-gyr.y(), gyr.x(), gyr.z());

      // Estimate attitude
      attitude_estimator_intel.update(enu_acc, enu_gyr, dt);
      attitude = attitude_estimator_intel.get_orientation();

      if(intel_count > 100){
        // Estimate position
        position_estimator_intel.update(enu_acc, attitude, dt, true);
        position = position_estimator_intel.position();

        // publis pose as TF
        tf_pub_intel_integration.publish(position, attitude);

      }
      intel_count++;
      /*
      madgwick_intel.update(enu_acc, enu_gyr, dt);

      //Vector3d nwu_acc(acc.z(), acc.x(), acc.y());
      Vector3d filt_acc = lpf.apply(enu_acc, 0.2);
      Vector3d linear_acc = custom_remove_gravity(filt_acc, madgwick_intel.get_orientation(), Vector3d(0, 0, 9.80665));
      
      // Pose estimator - EKF
      pose_estimator_intel.predict(dt);

      Vector3d pose_intel = pose_estimator_intel.get_position();
      //printf("Update with acc: [%.3f, %.3f %.3f]\n", linear_acc[0], linear_acc[1], linear_acc[2]);
      //pose_estimator_intel.print_state_vector();
      pose_estimator_intel.update(linear_acc);
      //pose_estimator_intel.print_state_vector();


      // Publish pose (EKF + Madgwick)
      tf_pub_intel.publish(pose_intel, madgwick_intel.get_orientation());
      
      // Publish orientation of imu
      tf_pub_intel_orientation.publish(static_pose_intel, madgwick_intel.get_orientation());

      // Publish linear acceleration vector
      imu_pub_intel_acc.publish(linear_acc, gyr);


      
      if (intel_count>0){

        pose_integrate.update(linear_acc, madgwick_intel.get_orientation(), dt);
        tf_pub_intel_integration.publish(pose_integrate.pose(), madgwick_intel.get_orientation());

      }
      intel_count++;
      */
    }

    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  return 0;
}

class IMUSubscriber{
 public:
  IMUSubscriber(){
    printf("TODO");
  }

  bool has_new_data(){
    return reader_.has_new_data();
  }
 private:
  IMU_Reader reader_;
};

int main_test_gravity(int argc, char **argv){
    ros::init(argc, argv, "IMU_TEST");
  ros::start();

  ros::NodeHandle n;

  // --- Readers ---
  IMU_Reader reader_T265;

  // --- Subscribers ---
  // T265
  message_filters::Subscriber<sensor_msgs::Imu>  acc_sub(n, "/T265/accel/sample", 100);
  message_filters::Subscriber<sensor_msgs::Imu> gyro_sub(n, "/T265/gyro/sample", 100);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(100), acc_sub, gyro_sub);
  sync.registerCallback(boost::bind(&IMU_Reader::read_acc_and_gyro_msg, &reader_T265, _1, _2));

  int FPS = 30;
  double last_t_T265=0.0;
  int count_T265 = 1;

  Madgwick madgwick_raw_acc =   Madgwick(0.0085); 
  Madgwick madgwick_filt =   Madgwick(0.0085); 
  LowPassFilter lpf_pre =  LowPassFilter(3, 0.25);
  LowPassFilter lpf_post = LowPassFilter(3, 0.25);
  Vector3d acc_filt_pre;
  Vector3d acc_filt_post;
  Vector3d linear_acc;
  Vector3d filt_acc;

  Vector3d g(0,0,10.136);

  Imu_Publisher pub_raw_linear_acc = Imu_Publisher(n, "odom", "T265_acc_raw");
  Imu_Publisher pub_linear_acc_post_filter = Imu_Publisher(n, "odom", "T265_acc_filt_post");
  Imu_Publisher pub_linear_acc_pre_filter = Imu_Publisher(n, "odom", "T265_acc_filt_pre");
  double alpha = 0.2;

  
    cout << "Imu node initialized correctly for test gravity with T265 dataset." << std::endl;

  ros::Rate r(30);
  while (ros::ok()) {

    if (reader_T265.has_new_data()) {
      IMU_Measurements imu_data = reader_T265.get_data();

      double t = imu_data.timestamp();
      double dt = t - last_t_T265;
      last_t_T265 = t;

      if (dt == t) // first iteration
        continue; 

      printf("[T265 count: %i]\n", count_T265);
      count_T265++;

      // Update orientation with measurements
      Vector3d acc = imu_data.acceleration();
      Vector3d gyr = imu_data.angular_velocity();
      // NWU to ENU
      Vector3d raw_acc(-acc.x(), acc.z(), acc.y());
      Vector3d raw_gyr(-gyr.x(), gyr.z(), gyr.y());
      printf("T265 measure ENU : (x: %.3f, y: %.3f, z: %.3f)\n", raw_acc.x(), raw_acc.y(), raw_acc.z());


      // linear acc form Raw acc
      madgwick_raw_acc.update(raw_acc, raw_gyr, dt);
      linear_acc = custom_remove_gravity(raw_acc, madgwick_raw_acc.get_orientation(), g);
      pub_raw_linear_acc.publish(linear_acc, raw_gyr, reader_T265.get_ros_time());

      // linear acc filter after remove grtavity
      filt_acc = lpf_post.apply(linear_acc);
      pub_linear_acc_post_filter.publish(linear_acc, raw_gyr, reader_T265.get_ros_time());


      // linear acc from pre filt acc
      filt_acc = lpf_pre.apply(raw_acc);
      linear_acc = custom_remove_gravity(filt_acc, madgwick_raw_acc.get_orientation(), g);
      pub_linear_acc_pre_filter.publish(linear_acc, raw_gyr, reader_T265.get_ros_time());

    }


    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  return 0;
}


int noise_T265(int argc, char **argv){
  ros::init(argc, argv, "IMU_TEST");
  ros::start();

  ros::NodeHandle n;
  IMU_Reader reader_T265;

  message_filters::Subscriber<sensor_msgs::Imu>  acc_sub(n, "/T265/accel/sample", 100);
  message_filters::Subscriber<sensor_msgs::Imu> gyro_sub(n, "/T265/gyro/sample", 100);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(100), acc_sub, gyro_sub);
  sync.registerCallback(boost::bind(&IMU_Reader::read_acc_and_gyro_msg, &reader_T265, _1, _2));

  Madgwick madgwick_T265 =   Madgwick(0.0085); 

  int FPS = 30;
  double last_t_T265=0.0;
  int count_T265 = 1;


  int max_size = 1000;
  int data_count = 0;
  Vector3d g(0,0,10.136);
  Vector3d linear_acc;
  Vector3d filt_acc;
  MatrixXd acc_raw_data(max_size, 3);
  MatrixXd acc_filtered_data(max_size, 3);
  LowPassFilter lpf = LowPassFilter(3, 0.25);
  double alpha = 0.2;

  cout << "Imu node initialized correctly for test noise of T265's accelerometer." << std::endl;

  ros::Rate r(100);
  while (ros::ok()) {

    if (reader_T265.has_new_data()) {
      IMU_Measurements imu_data = reader_T265.get_data();

      double t = imu_data.timestamp();
      double dt = t - last_t_T265;
      last_t_T265 = t;
      if (dt == t) // first iteration
        continue; 

      printf("[T265 count: %i]\n", count_T265);
      count_T265++;

      // Update orientation with measurements
      Vector3d acc = imu_data.acceleration();
      Vector3d gyr = imu_data.angular_velocity();
      // NWU to ENU
      Vector3d raw_acc(-acc.x(), acc.z(), acc.y());
      Vector3d raw_gyr(-gyr.x(), gyr.z(), gyr.y());

      // linear acc form Raw acc
      madgwick_T265.update(raw_acc, raw_gyr, dt);
      linear_acc = custom_remove_gravity(raw_acc, madgwick_T265.get_orientation(), g);
      
      if (count_T265>150){
        if(data_count < max_size){
          acc_raw_data.block<1,3>(data_count, 0) = raw_acc;
          filt_acc = lpf.apply(acc_raw_data);
          acc_filtered_data.block<1,3>(data_count, 0) = filt_acc;
          data_count++;
        }
        else{
          printf(" Implementar metodo para calcular media y std");
          break;
        }
      }

    }


    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  return 0;
}

Vector3d calibrate_gravity(Vector3d acc, Quaterniond  orientation){  

  orientation.normalize();

  Vector3d g = orientation * acc;
  Vector3d diff = acc - g;
  printf("Sensor measurement:  (x: %.3f, y: %.3f, z: %.3f)\n", acc.x(), acc.y(), acc.z());
  printf("Gravity estimate:    (x: %.3f, y: %.3f, z: %.3f)\n", g.x(), g.y(), g.z());
  printf("Difference:          (x: %.3f, y: %.3f, z: %.3f)\n", diff.x(), diff.y(), diff.z());
  return g;
}

/*
  Orientation and acceleration must be in the same coordinate system.
  * The orientation generated by madgwick is in the NWU
*/
Vector3d custom_remove_gravity(const Vector3d & acc, const Quaterniond & attitude, const Vector3d & g){  

  Quaterniond attitude_norm = attitude.normalized();
  //Vector3d g(0, 0, 10.12/*9.80665*/);

  Vector3d g_rot = attitude_norm.inverse() * g;
  Vector3d linear_acc = acc - g_rot;

  printf("Attitude:  (x: %.3f, y: %.3f, z: %.3f, w: %.3f)\n", attitude.x(), attitude.y(), attitude.z(), attitude.w());
  printf("Sensor measure:  (x: %.3f, y: %.3f, z: %.3f)\n", acc.x(), acc.y(), acc.z());
  printf("Gravity rotate:  (x: %.3f, y: %.3f, z: %.3f)\n", g_rot.x(), g_rot.y(), g_rot.z());
  printf("Linear acceler:  (x: %.3f, y: %.3f, z: %.3f)\n", linear_acc.x(), linear_acc.y(), linear_acc.z());
  
  return linear_acc;
}


void rotate_acc_experiment(Vector3d acc, Quaterniond  orientation){  

  orientation.normalize();
  double g = 10.136;
  Vector3d acc_rot = orientation.conjugate() * acc;
  Vector3d acc_odd_g = Vector3d(acc_rot.x(), acc_rot.y(), acc_rot.z() - g);

  printf("Sensor measure:  (x: %.3f, y: %.3f, z: %.3f)\n", acc.x(), acc.y(), acc.z());
  printf("Accel.  rotate:  (x: %.3f, y: %.3f, z: %.3f)\n", acc_rot.x(), acc_rot.y(), acc_rot.z());
  printf("      - %.3f:  (x: %.3f, y: %.3f, z: %.3f)\n", g, acc_odd_g.x(), acc_odd_g.y(), acc_odd_g.z());

}




class ZeroVelocityUpdate{
 private:
  double _th1; 
  double _th2;
  int _window_size;
  Vector3d _position;
  Vector3d _velocity;

 public:
  ZeroVelocityUpdate(double th1, double th2, double window_size){
    _th1 = th1;
    _th2 = th2;
    _window_size = window_size;
    _position.setZero();
    _velocity.setZero();
  }

  void update(Vector3d acc, double dt){
    // 1
    double acc_norm = acc.norm();

    // 2
    double acc_variance;  
  }


};

class PublisherData{
 private:
  TF_Publisher tf_pub_pose;
  
  // debug
  TF_Publisher tf_pub_attitude;
  Imu_Publisher imu_pub_filt_acc;
  Imu_Publisher imu_pub_linear_acc;

 public:
  PublisherData(ros::NodeHandle & n, 
                const string & base_frame, 
                const string & data_name):
    tf_pub_pose(TF_Publisher(n, base_frame, "pose_" + data_name)),
    tf_pub_attitude(TF_Publisher(n, base_frame, "attitude_" + data_name)),
    imu_pub_filt_acc(Imu_Publisher(n, base_frame, "debug_filt_acc_" + data_name)),
    imu_pub_linear_acc(Imu_Publisher(n, base_frame, "debug_linear_acc_" + data_name))
  {}

  void publish(PositionEstimator & position_estimator,
               Madgwick & attitude_estimator,
               const ros::Time & ros_time=ros::Time::now(),
               const bool publish_debug=false){
    
    tf_pub_pose.publish(position_estimator.position(), attitude_estimator.get_orientation());

    if (publish_debug){
      Vector3d zero_vec = Vector3d(0,0,0);
      tf_pub_attitude.publish(zero_vec, attitude_estimator.get_orientation());
    }
  }
}; 



int test_T265(int argc, char **argv) {
  ros::init(argc, argv, "IMU_TEST");
  ros::start();
  ros::NodeHandle n;

  // --- Readers ---
  IMU_Reader reader_arduimu;
  IMU_Reader reader_T265;

  // --- Subscribers ---
  // Arduimu (/imu_101)
  ros::Subscriber imu_sub = n.subscribe("/imu_101", 100, &IMU_Reader::read_imu_msg, &reader_arduimu);
  // T265
  message_filters::Subscriber<sensor_msgs::Imu>  acc_sub(n, "/T265/accel/sample", 100);
  message_filters::Subscriber<sensor_msgs::Imu> gyro_sub(n, "/T265/gyro/sample", 100);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(100), acc_sub, gyro_sub);
  sync.registerCallback(boost::bind(&IMU_Reader::read_acc_and_gyro_msg, &reader_T265, _1, _2));

  // --- Publishers ---
  string base_frame = "T265_odom_frame";
  PublisherData pub_T265 = PublisherData(n, base_frame, "T265");
  PublisherData pub_arduimu = PublisherData(n, "odom", "Arduimu");

  // --- Estimators ---
  PositionEstimator position_estimator_T265 = PositionEstimator(0.15, Vector3d(0.0, 0.0, 10.136));
  PositionEstimator position_estimator_arduimu = PositionEstimator(0.25);

  Madgwick attitude_estimator_T265 = Madgwick(0.0085); 
  //attitude_estimator_T265.set_orientation(Quaterniond(0.999, -0.043, 0.004, -0.012));
  Madgwick attitude_estimator_arduimu = Madgwick(0.01); 
  
  Madgwick attitude_estimator_T265_nwu = Madgwick(0.0085);

  TF_Publisher att_nwu = TF_Publisher(n, base_frame, "NWU"); 
  TF_Publisher att_enu = TF_Publisher(n, base_frame, "ENU"); 

  ImuFilter att_nwu_ros = ImuFilter(0.0085, 0.5);
  ImuFilter att_enu_ros = ImuFilter(0.0085, 0.5);
  TF_Publisher pub_att_nwu_ros = TF_Publisher(n, base_frame, "NWU_ros"); 
  TF_Publisher pub_att_enu_ros = TF_Publisher(n, base_frame, "ENU_ros"); 

  // --- CONFIG ---
  bool use_T265 = true;
  bool verbose_T265 = true;
  int th_T265 = 150;

  bool use_arduimu = false;


  printf("Imu node initialized correctly for test gravity with T265 dataset.\n");
  printf("\t Rate: 30 Hz\n");
  printf("\t Use T265 IMU: %s\n", use_T265 ? "YES" : "NO");
  printf("\t Use Arduimu IMU: %s\n\n", use_arduimu ? "YES" : "NO");


  double last_t_arduimu=0.0;
  double last_t_T265=0.0;
  int count_T265 = 1;
  Vector3d position;
  Quaterniond attitude;

  ros::Rate r(30);
  while (ros::ok()) {

    // T265 (WUN)
    if (reader_T265.has_new_data() && use_T265) {
      IMU_Measurements imu_data = reader_T265.get_data();

      // Update dt
      double dt = imu_data.timestamp() - last_t_T265;
      last_t_T265 = imu_data.timestamp();
      if (dt == imu_data.timestamp()){continue;}// first iteration
        
      printf("[T265 count: %i]\n", count_T265);
      count_T265++;

      // Rotate IMU data from WUN to NWU
      Vector3d acc = imu_data.acceleration();
      Vector3d gyr = imu_data.angular_velocity();
      attitude_estimator_T265_nwu.update(Vector3d(acc.z(), acc.x(), acc.y()), 
                                         Vector3d(gyr.z(), gyr.x(), gyr.y()),
                                         dt);  
      attitude = attitude_estimator_T265_nwu.get_orientation();
      att_nwu.publish(Vector3d(0,0,0), attitude);

      
      // end test nwu
      if (count_T265 > th_T265){
        position_estimator_T265.update(Vector3d(acc.z(), acc.x(), acc.y()), attitude, dt, verbose_T265);
        position = position_estimator_T265.position();
      }

      pub_T265.publish(position_estimator_T265, attitude_estimator_T265_nwu, reader_T265.get_ros_time(), verbose_T265);
      

    }

    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  return 0;
}



int old_test_T265(int argc, char **argv) {
  ros::init(argc, argv, "IMU_TEST");
  ros::start();

  ros::NodeHandle n;

  // --- Readers ---
  IMU_Reader reader_arduimu;
  IMU_Reader reader_T265;

  // --- Subscribers ---
  // Arduimu (/imu_101)
  ros::Subscriber imu_sub = n.subscribe("/imu_101", 100, &IMU_Reader::read_imu_msg, &reader_arduimu);
  // T265
  message_filters::Subscriber<sensor_msgs::Imu>  acc_sub(n, "/T265/accel/sample", 100);
  message_filters::Subscriber<sensor_msgs::Imu> gyro_sub(n, "/T265/gyro/sample", 100);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(100), acc_sub, gyro_sub);
  sync.registerCallback(boost::bind(&IMU_Reader::read_acc_and_gyro_msg, &reader_T265, _1, _2));

  // --- TF Publisher ---
  TF_Publisher tf_pub_arduimu_attitude = TF_Publisher(n, "odom", "arduimu_attitude");
  TF_Publisher tf_pub_T265_attitude =   TF_Publisher(n, "odom", "T265_attitude");
  TF_Publisher tf_pub_arduimu = TF_Publisher(n, "odom", "arduimu_pose");
  TF_Publisher tf_pub_T265 =   TF_Publisher(n, "odom", "T265_pose_ekf");
  TF_Publisher tf_pub_T265_test =   TF_Publisher(n, "odom", "T265_pose_test");


  // Acc publisher
  Imu_Publisher imu_pub_arduino_acc = Imu_Publisher(n, "arduimu_attitude", "arduimu_acc");
  Imu_Publisher imu_pub_T265_acc =   Imu_Publisher(n, "T265_attitude", "T265_acc");
  Imu_Publisher imu_pub_T265_test =   Imu_Publisher(n, "T265_pose_test", "T265_acc_filt");


  // IMU orientation calculator
  Madgwick madgwick_arduino = Madgwick(0.01); 
  Madgwick madgwick_T265 =   Madgwick(0.0085); 
  madgwick_T265.set_orientation(Quaterniond(0.999, -0.043, 0.004, -0.012));
  
  // IMU pose estimator
  PoseEstimator pose_estimator_arduino = PoseEstimator();
  PoseEstimator pose_estimator_T265   = PoseEstimator();

  PoseEstimator pose_estimator_T265_test   = PoseEstimator();
  Odometry pose_integrate = Odometry(0.1);
  TF_Publisher tf_pub_T265_integration =   TF_Publisher(n, "odom", "T265_pose_integrate");

  // AUX
  Vector3d static_pose_arduino(0, -0.5, 0);
  Vector3d static_pose_T265  (0,  0.5, 0);

  Vector3d filter_acc(0,0,0);
  Vector3d last_acc(0,0,0);
  Vector3d last_sensor_acc(0,0,0);
  cout << "Imu node initialized correctly for test gravity with T265 dataset." << std::endl;

  int FPS = 30;
  double last_t_arduimu=0.0;
  double last_t_T265=0.0;
  int count_T265 = 1;

  int calib_size = 1000;
  int calib_intel = 0;
  MatrixXd cal(calib_size, 3);  
  //HighPassFilter hpf = HighPassFilter(3);
  LowPassFilter lpf = LowPassFilter(3, 0.25);
  LowPassFilter lpf2 = LowPassFilter(3, 0.25);

  ros::Rate r(30);
  while (ros::ok()) {

    // T265 (WUN)
    if (reader_T265.has_new_data()) {
      IMU_Measurements imu_data = reader_T265.get_data();

      double t = imu_data.timestamp();
      double dt = t - last_t_T265;
      last_t_T265 = t;

      if (dt == t) // first iteration
        dt = 1 / FPS; 

      printf("[T265 count: %i]\n", count_T265);
      count_T265++;

      // Update orientation with measurements
      Vector3d acc = imu_data.acceleration();
      Vector3d gyr = imu_data.angular_velocity();
      printf("T265 measure: (x: %.3f, y: %.3f, z: %.3f)\n", acc.x(), acc.y(), acc.z());

      Vector3d enu_acc(-acc.x(), acc.z(), acc.y());
      Vector3d enu_gyr(-gyr.x(), gyr.z(), gyr.y());
      printf("T265 ENU : (x: %.3f, y: %.3f, z: %.3f)\n", enu_acc.x(), enu_acc.y(), enu_acc.z());
      madgwick_T265.update(enu_acc, enu_gyr, dt);

      double alpha_sensor = 0.1;

      Vector3d filt_acc = lpf.apply(enu_acc);
      //Vector3d filt_acc = enu_acc;
      Vector3d linear_acc = custom_remove_gravity(filt_acc, madgwick_T265.get_orientation(), Vector3d(0.0, 0.0, 10.136));

      // Publish orientation of imu
      tf_pub_T265_attitude.publish(static_pose_T265, madgwick_T265.get_orientation());

      // Publish linear acceleration vector
      imu_pub_T265_acc.publish(linear_acc, gyr, reader_T265.get_ros_time());



      /* TEST */
      if (count_T265>150){
        printf("Filter acceler:  [%.3f, %.3f %.3f]\n", linear_acc[0], linear_acc[1], linear_acc[2]);
        imu_pub_T265_test.publish(filter_acc, gyr, reader_T265.get_ros_time());

        // POSE ESTIMATOR - EKF
        /*
        pose_estimator_T265.predict(dt);
        pose_estimator_T265.print_state_vector();
        Vector3d position = pose_estimator_T265.get_position();
        Quaterniond attitude = madgwick_T265.get_orientation();
        tf_pub_T265.publish(position, attitude);
        pose_estimator_T265.update(linear_acc);
        */

        // POSE ESTIMATOR - Odometry
        //pose_integrate.update(linear_acc, madgwick_T265.get_orientation(), dt, 0.2);
        pose_integrate.update_3(linear_acc, madgwick_T265.get_orientation(), dt);
        tf_pub_T265_integration.publish(pose_integrate.pose(), madgwick_T265.get_orientation());
        
      }

      printf("\n");
    }

    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  return 0;
}


int main(int argc, char **argv) {
  if(argc != 2) {
      cerr << endl << "Usage: rosrun SD-SLAM IMU_TEST MODE\n Modes: 'd435i', 'grav_T265', 'T265'" << endl;
      ros::shutdown();
      return 1;
    }
  
  string mode = argv[1];
  if (mode == "d435i")
    return main_d435i(argc, argv);
  else if (mode == "T265")
    return test_T265(argc, argv);
  else if (mode == "grav_T265")
    return main_test_gravity(argc, argv);
  else if (mode == "noise_T265")
    return noise_T265(argc, argv);

  else
    cerr << endl << "Does not recognize selected MODE: 'd435i', 'GRAV', 'test_T265'" << argv[1] << endl;

  
 
  //return main_only_imu(argc, argv);
  
  //return main_EuRoC(argc, argv);
}
