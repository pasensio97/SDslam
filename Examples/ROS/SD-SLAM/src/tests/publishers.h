
#include <iostream>
#include <algorithm>
#include <fstream>
#include "src/inertial/IMU_Measurements.h"
#include "inertial/attitude_estimators/Madgwick.h"
#include "inertial/PositionEstimator.h"
#include "inertial/tools/filters.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class Imu_Publisher{
 private:
    ros::Publisher pub;
    string frame_id;
    int seq;

 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Imu_Publisher(ros::NodeHandle & nodehandle, const string &frame, const string &topic):
    pub(nodehandle.advertise<sensor_msgs::Imu>(topic, 100)), frame_id(frame), seq(0)
  {}
  
  void publish(const Vector3d & acc, 
               const Vector3d & gyr, 
               ros::Time timestamp=ros::Time::now());
};


class TF_Publisher {
 private:
  tf::TransformBroadcaster _br;

 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  const std::string base_frame;
  const std::string child_frame;


  TF_Publisher(ros::NodeHandle & n, const string & base_frame, const string & child_frame):
    base_frame(base_frame), child_frame(child_frame)
  {}

  void publish(Vector3d position, Quaterniond orientation);
};


class Odom_Publisher {
 private:
	ros::Publisher _pub;
	tf::TransformBroadcaster _br;
	const std::string _base_frame;
  const std::string _child_frame;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Odom_Publisher(ros::NodeHandle & n, const string & base_frame, const string & child_frame):
    _pub(n.advertise<nav_msgs::Odometry>(child_frame, 10)),
		_base_frame(base_frame), _child_frame(child_frame)
  {}

  void publish(const ros::Time & stamp, 
							 const Vector3d & position, const Quaterniond & orientation, 
							 const Vector3d linear_vel = Vector3d(0,0,0),
							 const Vector3d angular_vel = Vector3d(0,0,0));
};



class PublisherData{
 private:
  TF_Publisher tf_pub_pose;
  
  // debug
  TF_Publisher tf_pub_attitude;
  Imu_Publisher imu_pub_filt_acc;
  Imu_Publisher imu_pub_linear_acc;
  Imu_Publisher imu_pub_velocity;
  Imu_Publisher imu_pub_position;

 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PublisherData(ros::NodeHandle & n, 
                const string & base_frame, 
                const string & data_name):
    tf_pub_pose(TF_Publisher(n, base_frame, "pose_" + data_name)),
    tf_pub_attitude(TF_Publisher(n, base_frame, "attitude_" + data_name)),
    imu_pub_filt_acc(Imu_Publisher(n, base_frame, "debug_filt_acc_" + data_name)),
    imu_pub_linear_acc(Imu_Publisher(n, base_frame, "debug_linear_acc_" + data_name)),
    imu_pub_velocity(Imu_Publisher(n, base_frame, "debug_velocity_" + data_name)),
    imu_pub_position(Imu_Publisher(n, base_frame, "debug_position_" + data_name))
  {}

  void publish(PositionEstimator & position_estimator,
               Madgwick & attitude_estimator,
               const ros::Time & ros_time=ros::Time::now(),
               const bool publish_debug=false){
    
    tf_pub_pose.publish(position_estimator.position(), attitude_estimator.get_orientation());

    if (publish_debug){
      Vector3d zero_vec = Vector3d(0,0,0);
      tf_pub_attitude.publish(zero_vec, attitude_estimator.get_orientation());
      imu_pub_velocity.publish(position_estimator.velocity(), zero_vec);
      imu_pub_position.publish(position_estimator.position(), zero_vec);
    }
  }
}; 
