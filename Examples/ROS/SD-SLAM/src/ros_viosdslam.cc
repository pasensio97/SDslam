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

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>

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

    vector<double> imu;
    imu.push_back(msgIMU->angular_velocity.x);
    imu.push_back(msgIMU->angular_velocity.y);
    imu.push_back(msgIMU->angular_velocity.z);
    imu.push_back(msgIMU->linear_acceleration.x);
    imu.push_back(msgIMU->linear_acceleration.y);
    imu.push_back(msgIMU->linear_acceleration.z);

    ROS_INFO("Read new %dx%d image", cv_ptrRGB->image.cols, cv_ptrRGB->image.rows);

    {
      std::unique_lock<mutex> lock(imgMutex_);
      cv_ptrRGB->image.copyTo(imgRGB_);
      channels_ = imgRGB_.channels();
      IMU_ = imu;
      updated_ = true;
    }
  }

  void GetData(cv::Mat &imgRGB, vector<double> &IMU) {
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
  vector<double> IMU_;
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
  cv::Mat im_rgb, im;
  vector<double> imu;
  bool useViewer = true;

  ros::init(argc, argv, "Vio_SDslam");
  ros::start();

  if(argc != 3) {
    cerr << endl << "Usage: rosrun SD-SLAM Vio_SDslam path_to_rosbag path_to_settings" << endl;
    ros::shutdown();
    return 1;
  }

  // Read rosbag
  string rosbag_path = argv[1];

  // Read parameters
  SD_SLAM::Config &config = SD_SLAM::Config::GetInstance();
  if (!config.ReadParameters(argv[2])) {
    cerr << "[ERROR] Config file contains errors" << endl;
    ros::shutdown();
    return 1;
  }

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

  // Subscribe to topics
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, config.CameraTopic(), 1);
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(n, config.IMUTopic(), 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Imu> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, imu_sub);
  sync.registerCallback(boost::bind(&ImageReader::ReadRGBIMU, &reader, _1, _2));

  double dt = 0.1;
  uint ni = 0;

  // // Main loop
  // ros::Rate r(30);
  // while (ros::ok() && !SLAM.StopRequested()) {
  //   if (reader.HasNewImage()) {
  //     // Get new image
  //     if (reader.NumChannels() == 1) {
  //       reader.GetData(im, imu);
  //     } else {
  //       reader.GetData(im_rgb, imu);
  //       cv::cvtColor(im_rgb, im, CV_RGB2GRAY);
  //     }

  //     // Ignore first frames to
  //     if (ni < 4) { im.setTo(cv::Scalar(0)); }
  //     // Force lost
  //     if (ni> 390  && ni<=445) { im.setTo(cv::Scalar(0)); }

  //     // Pass the image and IMU data to the SLAM system
  //     Eigen::Matrix4d pose = SLAM.TrackNewFusion(im, imu, dt, timestamps[ni]);

  //     // Show world pose
  //     ShowPose(pose);

  //     // Set data to UI
  //     fdrawer->Update(im, pose, tracker);
  //     mdrawer->SetCurrentCameraPose(pose);
  //   }

  //   ros::spinOnce();
  //   r.sleep();

  //   if (useViewer && viewer->isFinished()) {
  //     ros::shutdown();
  //     return 0;
  //   }
  //   ni++;
  // }

  rosbag::Bag bag;
  bag.open(rosbag_path);  // BagMode is Read by default

  ros::Rate r(30);
  for(rosbag::MessageInstance const m: rosbag::View(bag) )
  {
    if (ni > 300)
      break;

    std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
    if (i != nullptr)
      std::cout << i->data << std::endl;
    std::cout << m.getTime() << '\t' << m.getTopic() << std::endl;

    r.sleep();
    ni++;
  }

  bag.close();

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
