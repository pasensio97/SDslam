

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include "System.h"
#include "Tracking.h"
#include "Map.h"
#include "Config.h"
#include "extra/timer.h"
#include "ui/Viewer.h"
#include "ui/FrameDrawer.h"
#include "ui/MapDrawer.h"
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
#include <random>
#include "tests/euroc_reader.h"

using namespace std;

const int SEED = 42;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "EUROC_TEST");
  ros::start();
  ros::NodeHandle n;    

  const string SEQ_PATH = "/media/javi/Datos/TFM/datasets/EuRoC/V1_02/mav0/";
  //"/media/javi/Datos/TFM/datasets/EuRoC/MH_05/files/";
  //string seq = string(argv[2]);

  EuRoC_Reader* reader = new EuRoC_Reader(SEQ_PATH);
  bool use_synthetic_acc = true;
  double dt = 0.0;
  double last_t = reader->current_img_data().timestamp;

  double mean = 0.0; double std = 0.0;
  if (mean != 0.0 or std != 0.0){
    reader->add_noise_to_synthetic_acc(mean, std, SEED);
  }
  
  // Use SDSLAM
  SD_SLAM::Config &config = SD_SLAM::Config::GetInstance();
  if (!config.ReadParameters(argv[1])) {
    cerr << "[ERROR] Config file contains errors" << endl;
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

  viewer = new SD_SLAM::Viewer(&SLAM, fdrawer, mdrawer);
  tviewer = new std::thread(&SD_SLAM::Viewer::Run, viewer);

  tracker->model_type = "imu_s";
  tracker->new_imu_model.set_remove_gravity_flag(false);
    

  cv::Mat img;
  EuRoC_ImgData img_data;
  IMU_Measurements imu;
  Matrix4d identity_4d = Matrix4d::Identity();
  int count = 1; int max_count = reader->total_images();

  // --- MAIN LOOP ---
  while (ros::ok() && reader->has_next()) {
    printf("%d / %d\n", count, max_count); count++;
    reader->next();

    img_data = reader->current_img_data();
    imu = reader->current_imu_data();
    dt = img_data.timestamp - last_t; last_t = img_data.timestamp;

    string filename = img_data.filename; filename.pop_back(); // .... idk
    img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    if (count> 300 && count<=320){img.setTo(cv::Scalar(0));}
    if(img.empty()) {
      cerr << endl << "[ERROR] Failed to load image at: "  << img_data.filename << endl;
      return 1;
    }
    
    
    // ------ SLAM
    SD_SLAM::Timer ttracking(true);
    // Pass the image and measurements to the SLAM system
    Eigen::Matrix4d pose = SLAM.TrackNewFusion(img, imu, dt, img_data.timestamp); 

    // Set data to UI    
    fdrawer->Update(img, pose, tracker);
    mdrawer->SetCurrentCameraPose(pose, tracker->used_imu_model);
    // ------ end SLAM


    // Wait to load the next frame
    ttracking.Stop();
    double delay = ttracking.GetTime();

    // Wait to load the next frame
    if(delay<dt)
      usleep((dt-delay)*1e6);
    //usleep(2.5e5);
    if (viewer->isFinished()){
      return 0;
    }
   
  }

  // Stop all threads
  SLAM.Shutdown();

  //string tum_file = "/home/javi/tfm/TEMP_FILES/kitti_" + kitti_seq.SEQ + ".txt";
  //SLAM.save_as_tum(tum_file);

  while (!viewer->isFinished())
    usleep(5000);
  viewer->RequestFinish();
  while (!viewer->isFinished())
    usleep(5000);
  tviewer->join();


  ros::shutdown();
  return 0;

}


