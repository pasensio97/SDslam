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
#include "extra/timer.h"
#include "Converter.h"
#include "inertial/IMU_Measurements.h"
#include "Examples/Help/kittiRawReader.h"
#include <random>
#include <Eigen/StdVector>


int main(int argc, char **argv) {
  vector<string> vFilenames;
  cv::Mat im_rgb, im;
  vector<double> imu;
  bool useViewer = true;

  ros::init(argc, argv, "Vio-SDslam");
  ros::start();

  if(argc != 2) {
    cerr << endl << "Usage: rosrun SD-SLAM Vio-SDslam path_to_settings" << endl;
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
  SD_SLAM::System SLAM(SD_SLAM::System::MONOCULAR_IMU, true);

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
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(n, config.IMUTopic(), 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Imu> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, imu_sub);
  sync.registerCallback(boost::bind(&ImageReader::ReadRGBIMU, &reader, _1, _2));

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

      // Pass the image and IMU data to the SLAM system
      Eigen::Matrix4d pose = SLAM.TrackFusion(im, imu);

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
