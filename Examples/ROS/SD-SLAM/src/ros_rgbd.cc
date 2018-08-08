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
#include "System.h"
#include "Tracking.h"
#include "Map.h"
#include "Config.h"
#include "ui/Viewer.h"
#include "ui/FrameDrawer.h"
#include "ui/MapDrawer.h"

using namespace std;

class ImageReader {
 public:
  ImageReader() {
    channels_ = 0;
    updated_ = false;
  }

  void ReadRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try {
      cv_ptrD = cv_bridge::toCvShare(msgD);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


    ROS_INFO("Read new %dx%d image", cv_ptrRGB->image.cols, cv_ptrRGB->image.rows);

    {
      std::unique_lock<mutex> lock(imgMutex_);
      cv_ptrRGB->image.copyTo(imgRGB_);
      cv_ptrD->image.copyTo(imgD_);
      channels_ = imgRGB_.channels();
      updated_ = true;
    }
  }

  void GetImage(cv::Mat &imgRGB, cv::Mat &imgD) {
    std::unique_lock<mutex> lock(imgMutex_);
    imgRGB_.copyTo(imgRGB);
    imgD_.copyTo(imgD);
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
  cv::Mat imgD_;
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

int main(int argc, char **argv) {
  vector<string> vFilenames;
  cv::Mat im_rgb, im, imD;
  bool useViewer = true;

  ros::init(argc, argv, "Monocular");
  ros::start();

  if(argc != 2) {
    cerr << endl << "Usage: rosrun SD-SLAM RGBD path_to_settings" << endl;
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

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  SD_SLAM::System SLAM(SD_SLAM::System::RGBD, true);

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

  // Subscribe to topics
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, config.CameraTopic(), 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, config.DepthTopic(), 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
  sync.registerCallback(boost::bind(&ImageReader::ReadRGBD, &reader, _1, _2));

  ros::Rate r(30);
  while (ros::ok() && !SLAM.StopRequested()) {
    if (reader.HasNewImage()) {
      // Get new image
      if (reader.NumChannels() == 1) {
        reader.GetImage(im, imD);
      } else {
        reader.GetImage(im_rgb, imD);
        cv::cvtColor(im_rgb, im, CV_RGB2GRAY);
      }

      // Pass the image to the SLAM system
      Eigen::Matrix4d pose = SLAM.TrackRGBD(im, imD);

      // Show world pose
      ShowPose(pose);

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
