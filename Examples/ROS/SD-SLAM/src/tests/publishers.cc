

#include <iostream>
#include <algorithm>
#include <fstream>
#include "inertial/IMU_Measurements.h"
#include "inertial/attitude_estimators/Madgwick.h"
#include "inertial/PositionEstimator.h"
#include "inertial/tools/filters.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/time_synchronizer.h>
#include "src/tests/publishers.h"

using namespace std;

void Imu_Publisher::publish(const Vector3d & acc, 
                            const Vector3d & gyr, 
                            ros::Time timestamp){
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


void TF_Publisher::publish(Vector3d position, Quaterniond orientation){

  ros::Time time_now = ros::Time::now();

  tf::Vector3 tf_position(position.x(), position.y(), position.z());
  tf::Quaternion tf_orientation(orientation.x(), orientation.y(), orientation.z(), orientation.w()); 

  tf::Transform tf;
  tf.setOrigin(tf_position);  
  tf.setRotation(tf_orientation);

  tf::StampedTransform stamped_transform = tf::StampedTransform(tf, time_now, base_frame, child_frame);
  _br.sendTransform(stamped_transform);
}


void Odom_Publisher::publish(const ros::Time & stamp, 
							 const Vector3d & position, const Quaterniond & orientation, 
							 const Vector3d linear_vel,
							 const Vector3d angular_vel){
    
    
    geometry_msgs::Quaternion odom_orientation;
    odom_orientation.x = orientation.x();
    odom_orientation.y = orientation.y();
    odom_orientation.z = orientation.z();
    odom_orientation.w = orientation.w();
    
    // TF
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = _base_frame;
    odom_trans.child_frame_id = _child_frame;

    odom_trans.transform.translation.x = position.x();
    odom_trans.transform.translation.y = position.y();
    odom_trans.transform.translation.z = position.z();
    odom_trans.transform.rotation = odom_orientation;

    _br.sendTransform(odom_trans);

    // Odom
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = _base_frame;

    odom.pose.pose.position.x = position.x();
    odom.pose.pose.position.y = position.y();
    odom.pose.pose.position.z = position.z();
    odom.pose.pose.orientation = odom_orientation;

    odom.child_frame_id = _child_frame;
    odom.twist.twist.linear.x = linear_vel.x();
    odom.twist.twist.linear.y = linear_vel.y();
    odom.twist.twist.linear.z = linear_vel.z();
    odom.twist.twist.angular.x = angular_vel.x();
    odom.twist.twist.angular.y = angular_vel.y();
    odom.twist.twist.angular.z = angular_vel.z();

    _pub.publish(odom);

}