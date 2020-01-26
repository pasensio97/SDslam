/**
 *
 *  Copyright (C) 2018 Eduardo Perdices <eperdices at gsyc dot es>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

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
#include "inertial/IMU_Measurements.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class ImageReader {
 public:
  ImageReader() {
    channels_ = 0;
    updated_ = false;
  }

  void ReadRGBIMU(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImuConstPtr& msgIMU) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Vector3d gyro(msgIMU->angular_velocity.x,
                  msgIMU->angular_velocity.y,
                  msgIMU->angular_velocity.z);
    Vector3d acce(msgIMU->linear_acceleration.x,
                  msgIMU->linear_acceleration.y,
                  msgIMU->linear_acceleration.z);
    double timestamp = msgIMU->header.stamp.toSec();

    IMU_Measurements imu = IMU_Measurements(timestamp, acce, gyro);

    ROS_INFO("Read new %dx%d image", cv_ptrRGB->image.cols, cv_ptrRGB->image.rows);

    {
      std::unique_lock<mutex> lock(imgMutex_);
      cv_ptrRGB->image.copyTo(imgRGB_);
      channels_ = imgRGB_.channels();
      IMU_ = imu;
      updated_ = true;
    }
  }

  void GetData(cv::Mat &imgRGB, IMU_Measurements &IMU) {
    std::unique_lock<mutex> lock(imgMutex_);
    imgRGB_.copyTo(imgRGB);
    IMU = IMU_;
    updated_ = false;
  }

  bool HasNewImage() {
    return updated_;
  }

  int NumChannels() {
    return channels_;
  }

 private:
  bool updated_;
  cv::Mat imgRGB_;
  IMU_Measurements IMU_;
  int channels_;
  std::mutex imgMutex_;
};

void ShowPose(const Eigen::Matrix4d &pose) {
  Eigen::Matrix4d wpose;
  wpose.setIdentity();
  wpose.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0).transpose();
  wpose.block<3, 1>(0, 3) = -pose.block<3, 3>(0, 0).transpose()*pose.block<3, 1>(0, 3);

  Eigen::Quaterniond q(wpose.block<3, 3>(0, 0));
  cout << "[INFO] World pose: [" << wpose(0, 3) << " " << wpose(1, 3) << " " << wpose(2, 3) << "]";
  cout << "[" << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "]" << endl;
}

class ROSPublisher {
 private:
  tf::TransformBroadcaster _br;

 public:

  ROSPublisher(ros::NodeHandle n){}
  void publish_orientation(const Eigen::Quaterniond q, ros::Time time, string base_frame, string local_frame){
    tf::Vector3 tf_position(0,0,0);
    tf::Quaternion tf_quaternion(q.x(), q.y(), q.z(), q.w());

    tf::Transform tr;
    tr.setOrigin(tf_position);
    tr.setRotation(tf_quaternion);
    tf::StampedTransform stamped_tr = tf::StampedTransform(tr, time, base_frame, local_frame);
    _br.sendTransform(stamped_tr);
    
  }
};


Quaterniond camera_to_world(const Matrix4d & pose){
  Matrix3d R;  // R = Rotx(-90)*Roty(90) | x->z; y->-x; z->-y
  R << 0,  0,  1,
      -1,  0,  0,
       0, -1,  0;

  Matrix3d pose_rot, w_pose;
  w_pose = pose.block<3, 3>(0, 0).transpose();
  pose_rot = R * w_pose * R.transpose();  // rot camera frame to NWU
  return Quaterniond(pose_rot);
}

int main(int argc, char **argv) {
  vector<string> vFilenames;
  cv::Mat im_rgb, im;
  IMU_Measurements imu;
  bool useViewer = true;

  ros::init(argc, argv, "Monocular");
  ros::start();

  if(argc < 2 and argc > 3) {
    cerr << endl << "Usage: rosrun SD-SLAM Inertial path_to_settings (gain)" << endl;
    ros::shutdown();
    return 1;
  }

  // Read parameters
  SD_SLAM::Config &config = SD_SLAM::Config::GetInstance();
  if (!config.ReadParameters(argv[1])) {
    cerr << "[ERROR] Config file contains errors" << endl;
    ros::shutdown();
    return 1;
  }

  double gain = (argc == 3) ?  atof(argv[2]) : 0.1;
  cout << "Madgwick gain: " << gain << endl;

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  SD_SLAM::System SLAM(SD_SLAM::System::MONOCULAR_IMU_NEW, true);

  // Create user interface
  SD_SLAM::Map * map = SLAM.GetMap();
  SD_SLAM::Tracking * tracker = SLAM.GetTracker();

  SD_SLAM::FrameDrawer * fdrawer = new SD_SLAM::FrameDrawer(map);
  SD_SLAM::MapDrawer * mdrawer = new SD_SLAM::MapDrawer(map);

  SD_SLAM::Viewer* viewer = nullptr;
  std::thread* tviewer = nullptr;

  if (useViewer) {
    viewer = new SD_SLAM::Viewer(&SLAM, fdrawer, mdrawer);
    tviewer = new std::thread(&SD_SLAM::Viewer::Run, viewer);
  }

  ros::NodeHandle n;
  ImageReader reader;
  ROSPublisher pub = ROSPublisher(n);

  // Subscribe to topics
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, config.CameraTopic(), 1);
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(n, config.IMUTopic(), 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Imu> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, imu_sub);
  sync.registerCallback(boost::bind(&ImageReader::ReadRGBIMU, &reader, _1, _2));

  double last_t = 0.0, dt = 0.0;
  SLAM.GetTracker()->set_madgwick_gain(gain);
  ros::Rate r(30);
  while (ros::ok() && !SLAM.StopRequested()) {
    if (reader.HasNewImage()) {
      // Get new image
      if (reader.NumChannels() == 1) {
        reader.GetData(im, imu);
      } else {
        reader.GetData(im_rgb, imu);
        cv::cvtColor(im_rgb, im, CV_RGB2GRAY);
      }

      dt = (last_t == 0.0) ? 0.0 : imu.timestamp() - last_t;
      last_t = imu.timestamp();
      
      // Pass the image and IMU data to the SLAM system
      Eigen::Matrix4d pose = SLAM.TrackNewFusion(im, imu, dt);

      // Show world pose
      ShowPose(pose);

      // Send tf
      ros::Time time_now = ros::Time::now();
      //  local
      //Quaterniond pred_mm  = SLAM.GetTracker()->pred_ctevel_q;
      //Quaterniond pred_mad = SLAM.GetTracker()->pred_mad_q;
      //  world
      Quaterniond world_prediction = camera_to_world(SLAM.GetTracker()->_prediction);
      Quaterniond world_cam = camera_to_world(pose);
      pub.publish_orientation(world_prediction, time_now, "odom", "pred");
      pub.publish_orientation(world_cam, time_now, "odom", "final");

      // Set data to UI
      fdrawer->Update(im, pose, tracker);
      mdrawer->SetCurrentCameraPose(pose);
    }

    ros::spinOnce();
    r.sleep();

    if (useViewer && viewer->isFinished()) {
      ros::shutdown();
      return 0;
    }
  }

  // Stop all threads
  SLAM.Shutdown();

  if (useViewer) {
    viewer->RequestFinish();
    while (!viewer->isFinished())
      usleep(5000);

    tviewer->join();
  }

  ros::shutdown();

  return 0;
}
