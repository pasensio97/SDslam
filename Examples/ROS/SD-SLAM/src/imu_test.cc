
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
#include "inertial/PoseEstimator.h"

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

    imu_data_ = IMU_Measurements(timestamp, acceleration, angular_velocity);
  }

  void read_acc_and_gyro_msg(const sensor_msgs::ImuConstPtr& msg_acc, const sensor_msgs::ImuConstPtr& msg_gyro) {
    updated_ = true;

    double acc_t =  msg_acc->header.stamp.toSec();
    double gyro_t =  msg_gyro->header.stamp.toSec();
    double timestamp = (acc_t > gyro_t) ? acc_t : gyro_t;

    Eigen::Vector3d acceleration = Eigen::Vector3d(msg_acc->linear_acceleration.x,
                                                   msg_acc->linear_acceleration.z, 
                                                   msg_acc->linear_acceleration.y);   
    Eigen::Vector3d angular_velocity = Eigen::Vector3d(msg_gyro->angular_velocity.x,
                                                       msg_gyro->angular_velocity.z, 
                                                       msg_gyro->angular_velocity.y);                                               

    imu_data_ = IMU_Measurements(timestamp, acceleration, angular_velocity);
  }

  IMU_Measurements get_data() {
    updated_ = false;
    return imu_data_;
  }

  bool has_new_data() {
    return updated_;
  }

 private:
  bool updated_;
  IMU_Measurements imu_data_;
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

class Odometry{
 private:
  Vector3d _pose = Vector3d(0, 0, 0);
  Vector3d _velocity = Vector3d(0, 0, 0);
  double _last_t;
  bool _initialize;
  double _global_t;

 public:
  Odometry(){
    _pose = Vector3d(0, 0, 0);
    _velocity = Vector3d(0, 0, 0);
    _last_t = 0.0;
    _initialize = false;
    _global_t = 0.0;
  }

  void update(Vector3d acc, Quaterniond orientation, double t){
    if (! _initialize){
      _last_t = t;
      _initialize = true;
    }
    else{
      double dt = t - _last_t;
      _last_t = t;
      
      Vector3d linear_acc = remove_gravity(acc, orientation);
      _velocity = _velocity + linear_acc * dt;
      _pose = _pose + _velocity * dt;

      /*
      double mod_v = sqrt(_velocity.x()*_velocity.x() + _velocity.y()*_velocity.y() + _velocity.z()*_velocity.z()); 
      double mod_a = sqrt(linear_acc.x()*linear_acc.x() + linear_acc.y()*linear_acc.y() + linear_acc.z()*linear_acc.z());

      std::cout <<  setprecision(6) << 
      "Acceleration: " << mod_v << "m/s^2 \n" <<
      "Velocity:     " << mod_v << "m/s, v(x: " << _velocity.x() << ", y: " << _velocity.y() << ", z: " << _velocity.z() << ")\n" << 
      "Position -> (x: " << _pose.x() << ", y: " << _pose.y() << ", z: " << _pose.z() << ")\n\n" << 
      std::endl;
      */
    }
  }

  Vector3d pose(){
    return Vector3d(_pose.x(), _pose.y(), _pose.z());
  }

  Vector3d remove_gravity(Vector3d acc, Quaterniond  &orientation, string cs = "NWU"){  
    Vector3d g(0, 0, 9.80665);
    if (cs == "ENU"){
      cout << "Transform acceleration from ENU to NWU to remove gravity.";
      acc = Vector3d(acc.y(), -acc.x(), acc.z());  // ENU to NWU
    }
    
    orientation.normalize();

    Vector3d g_rot = orientation.toRotationMatrix().inverse() * g;
    Vector3d linear_acc;
    linear_acc = acc - g_rot;
    /*
    std:: cout << setprecision(6) <<
    "Orientation is q(w: " << orientation.w() << ", x: " << orientation.x() << ", y: " << orientation.y() << ", z: " << orientation.z() << ")\n" <<
    //"Gravity vector rot. : (x: " << g_rot_2.x() << ", y: " << g_rot_2.y() << ", z: " << g_rot_2.z() << ")\n" <<  
    "Gravity vector rot. : (x: " << g_rot.x() << ", y: " << g_rot.y() << ", z: " << g_rot.z() << ")\n" <<  
    "Acceleration measu. : (x: " << acc.x() << ", y: " << acc.y() << ", z: " << acc.z() << ")\n" <<  
    "Linear acceleration:  (x: " << linear_acc.x() << ", y: " << linear_acc.y() << ", z: " << linear_acc.z() << ")" << 
    std::endl;
    */         
    return linear_acc;
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
  Orientation must stay in NWU coordinate system
*/
Vector3d remove_gravity(Vector3d acc, Quaterniond  orientation, string cs = "NWU"){  

  if (cs == "ENU"){
    cout << "Transform acceleration from ENU to NWU to remove gravity." << endl;
    acc = Vector3d(-acc.y(), -acc.x(), acc.z());  // ENU to NWU
  }
  else if (cs == "WUN"){  // d435i
    cout << "Transform acceleration from ENU to NWU to remove gravity." << endl;
    acc = Vector3d(-acc.y(), -acc.x(), acc.z());  // WUN to NWU
  }

  orientation.normalize();
  Vector3d g(0, 0, 9.80665);

  Vector3d g_rot = orientation.inverse() * acc;
  Vector3d linear_acc = acc - g_rot;

  double th = 100;
  if (linear_acc.x() > th || linear_acc.y() > th || linear_acc.z() > th ){
    printf("Error on calculate linear acc. \n\tG_acc: (%.4f, %.4f, %.4f)\n\tL_acc: (%.4f, %.4f, %.4f)",
    acc.x(), acc.y(), acc.z(), linear_acc.x(), linear_acc.y(), linear_acc.z());
    linear_acc = Vector3d(0,0,0);
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

  void publish(const Vector3d & acc, const Vector3d & gyr){
    sensor_msgs::Imu imu_msg;
    
    imu_msg.header.stamp = ros::Time::now();
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



  cout << "Imu node initialized correctly for D435i dataset." << std::endl;

  int FPS = 30;
  double last_t_arduimu=0.0;
  double last_t_intel=0.0;

  ros::Rate r(30);
  while (ros::ok()) {
    // Arduimu (ENU)
    /*
    if (reader_arduino.has_new_data()){
      IMU_Measurements imu_data = reader_arduino.get_data();

      double t = imu_data.timestamp();
      double dt = t - last_t_arduimu;
      last_t_arduimu = t;

      if (dt == t) // first iteration
        dt = 1 / FPS; 

      // Update orientation with measurements
      madgwick_arduino.update(imu_data.acceleration(), imu_data.angular_velocity(), dt);

      Vector3d grav_acc = imu_data.acceleration();
      Vector3d linear_acc = remove_gravity(grav_acc, madgwick_arduino.get_orientation(), "ENU");
      
      // Pose estimator - EKF
      pose_estimator_arduino.predict(dt);
      Vector3d pose_arduino = pose_estimator_arduino.get_position();
      pose_estimator_arduino.update(linear_acc);

      // Publish pose (EKF + Madgwick)
      tf_pub_arduino.publish(pose_arduino, madgwick_arduino.get_orientation());

      // Publish orientation of imu
      //tf_pub_arduino_orientation.publish(static_pose_arduino, madgwick_arduino.get_orientation());

      // Publish linear acceleration vector
      imu_pub_arduino_acc.publish(linear_acc, linear_acc);

    }
    */
    // D435i (WUN)
    if (reader_intel.has_new_data()) {
      IMU_Measurements imu_data = reader_intel.get_data();

      double t = imu_data.timestamp();
      double dt = t - last_t_intel;
      last_t_intel = t;

      if (dt == t) // first iteration
        dt = 1 / FPS; 

      // Update orientation with measurements
      Vector3d acc = imu_data.acceleration();
      Vector3d gyr = imu_data.angular_velocity();
      Vector3d enu_acc(acc.z(), -acc.x(), acc.y());
      Vector3d enu_gyr(gyr.z(), -gyr.x(), gyr.y());
      madgwick_intel.update(enu_acc, enu_gyr, dt);

      Vector3d nwu_acc(acc.z(), acc.x(), acc.y());
      Vector3d linear_acc = remove_gravity(enu_acc, madgwick_intel.get_orientation(), "ENU");
      
      // Pose estimator - EKF
      pose_estimator_intel.predict(dt);

      Vector3d pose_intel = pose_estimator_intel.get_position();
      printf("Update with acc: [%.3f, %.3f %.3f]\n", linear_acc[0], linear_acc[1], linear_acc[2]);
      pose_estimator_intel.print_state_vector();
      pose_estimator_intel.update(linear_acc);
      pose_estimator_intel.print_state_vector();


      // Publish pose (EKF + Madgwick)
      tf_pub_intel.publish(pose_intel, madgwick_intel.get_orientation());
      
      // Publish orientation of imu
      tf_pub_intel_orientation.publish(static_pose_intel, madgwick_intel.get_orientation());

      // Publish linear acceleration vector
      imu_pub_intel_acc.publish(linear_acc, gyr);
    }

    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  return 0;
}

int main(int argc, char **argv) {
  if(argc != 2) {
      cerr << endl << "Usage: rosrun SD-SLAM IMU_TEST MODE\n Modes: 'd435i'" << endl;
      ros::shutdown();
      return 1;
    }
  
  string mode = argv[1];
  cout << mode << endl;
  if (mode == "d435i")
    return main_d435i(argc, argv);

  else
    cerr << endl << "Does not recognize selected MODE: " << argv[1] << endl;

  
 
  //return main_only_imu(argc, argv);
  
  //return main_EuRoC(argc, argv);
}
