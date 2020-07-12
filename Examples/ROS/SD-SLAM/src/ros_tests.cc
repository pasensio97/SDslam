
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <ros/ros.h>
#include "inertial/IMU_Measurements.h"
#include "inertial/attitude_estimators/Madgwick.h"
#include "tests/kitti_help.h"
#include "tests/euroc_reader.h"
#include "tests/publishers.h"
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "System.h"
#include "Tracking.h"
#include "Map.h"
#include "Config.h"
#include "extra/timer.h"
#include "ui/Viewer.h"
#include "ui/FrameDrawer.h"
#include "ui/MapDrawer.h"
#include "inertial/position_estimators/AccIntegrator.h"

using namespace std;
using namespace Eigen;


vector<Vector3d> read_file(string & filename){
  ifstream file;
  file.open(filename.c_str());

  string line;
  vector<Vector3d> content;

  int it=0;
  while(getline(file, line)) { 
    if (it == 0){
      it++;
      continue; // header;  
    }
    std::stringstream  line_stream(line);
    
    vector<double> line_content;
    string svalue;
    while (getline(line_stream, svalue, ',')) {
        line_content.push_back(stod(svalue));
    }
 
    content.push_back(Vector3d(line_content[0], line_content[1], line_content[2]));
  }
  file.close();

  return content;
}


void test_kitti_imu(ros::NodeHandle &n, int max_its){
  printf("Executing test...\n");

  Kitti kitti;
  const string KITTI_SEQ = "/media/javi/Datos/TFM/datasets/kitti/raw/seq_00/2011_10_03/2011_10_03_drive_0027_sync";

  string kitti_gt = KITTI_SEQ + "/gt.txt";
  vector<Vector3d> gt_pos;
  vector<Quaterniond> gt_att;
  kitti.read_gps(kitti_gt, gt_pos, gt_att);

  string synthetic_acc_file = "/home/javi/tfm/tests/simulate_imu/temp_files/synthetic_acc.csv";
  string synthetic_gyr_file = "/home/javi/tfm/tests/simulate_imu/temp_files/synthetic_gyr.csv";
  auto syn_acc = read_file(synthetic_acc_file);
  auto syn_gyr = read_file(synthetic_gyr_file);

  string kitti_imu = KITTI_SEQ + "/oxts/data/";
  vector<string> imu_filenames = kitti.load_imu_filenames(kitti_imu);


  TF_Publisher gt_pub(n, "odom", "gt");
  TF_Publisher nwu_pub(n, "odom", "nwu_syn");
  TF_Publisher cam_pub(n, "odom", "cam_syn");
  TF_Publisher nwu_imu_pub(n, "odom", "nwu_imu");
  TF_Publisher cam_imu_pub(n, "odom", "cam_imu");
  Vector3d v_zeros(0,0,0);

  Madgwick madgwick_nwu(0.0085);
  Madgwick madgwick_cam(0.0085);
  Madgwick madgwick_imu_nwu(0.0085);
  Madgwick madgwick_imu_cam(0.0085);
  
  
  double dt = 0.1;
  if (max_its == 0){
    max_its = min(gt_att.size(), syn_acc.size());
  }
  for(int i=0; i<max_its; i++){
    madgwick_cam.update(syn_acc[i], syn_gyr[i], dt);

    auto syn_acc_nwu = kitti.rotation_imu_cam().inverse() * syn_acc[i];
    auto syn_gyr_nwu = kitti.rotation_imu_cam().inverse() * syn_gyr[i];
    madgwick_nwu.update(syn_acc_nwu, syn_gyr_nwu, dt);


    IMUSensor imu_data = kitti.read_imu_file(imu_filenames[i]);
    madgwick_imu_nwu.update(imu_data.acc_vehicle, imu_data.angular_vel_vehicle, dt);
    Matrix3d R_nwu2enu;
    R_nwu2enu << 0,-1, 0, 1, 0, 0, 0, 0, 1;
    madgwick_imu_cam.update(kitti.rotation_velo_cam() * (kitti.rotation_imu_velo() * imu_data.acc_vehicle),
                            kitti.rotation_velo_cam() * (kitti.rotation_imu_velo() * imu_data.angular_vel_vehicle),
                            dt);

    Quaterniond gt_inv = gt_att[i].conjugate(); // inv fails... idk why
    gt_pub.publish(v_zeros, gt_inv); // === SLAM
    nwu_imu_pub.publish(v_zeros, madgwick_imu_nwu.get_local_orientation()); // === SLAM
    nwu_pub.publish(v_zeros, madgwick_nwu.get_local_orientation());  // ? SLAM


    cam_pub.publish(v_zeros, madgwick_cam.get_orientation());
    cam_imu_pub.publish(v_zeros, madgwick_imu_cam.get_orientation().conjugate());

    usleep(1e5);
  }
  
}



void test_euroc(ros::NodeHandle &n, int max_its){
  const uint SEED = 42;
  const string SEQ_PATH = "/media/javi/Datos/TFM/datasets/EuRoC/MH_05/files/";
  printf("Executing EuRoC test usign seq: %s\n", SEQ_PATH.c_str());

  EuRoC_Reader* euroc_reader = new EuRoC_Reader(SEQ_PATH);
  bool use_synthetic_acc = true;
  double dt = 0.0;
  double last_t = euroc_reader->current_img_data().timestamp;

  AccIntegrator pos_est(!use_synthetic_acc);

  double POSITION_FACTOR = 0.5;
  // Generate acceleration from GT
  double mean = 0.0;
  double std = 0.0;
  if (mean != 0.0 or std != 0.0){
    euroc_reader->add_noise_to_synthetic_acc(mean, std, SEED);
  }
  
  // GT data and publisher
  EuRoC_GTData* gt_data;
  EuRoC_GTData* gt_T_data;
  Odom_Publisher odom_gt_real(n, "odom", "odom_gt_real");
  Odom_Publisher odom_gt_T(n, "odom", "odom_gt_T");
  Odom_Publisher odom_est(n, "odom", "odom_est");
  
  cv::Mat image;
  IMU_Measurements imu_data;
  EuRoC_ImgData img_data;
  
  cout << "STARTING!" << endl;
  int count = 1; int max_count = euroc_reader->total_images();

  while(euroc_reader->has_next()){
    printf("%d / %d\n", count, max_count); count++;

    euroc_reader->next();
    img_data = euroc_reader->current_img_data();
    imu_data = euroc_reader->current_imu_data(use_synthetic_acc);
    gt_data = euroc_reader->current_gt_data();
    gt_T_data = euroc_reader->current_gt_origin_data();
    
    dt = img_data.timestamp - last_t; last_t = img_data.timestamp;
    image = cv::imread(img_data.filename, CV_LOAD_IMAGE_GRAYSCALE);

    
    Vector3d position = pos_est.update(imu_data.acceleration(), Quaterniond(), dt);
    cout << "Acc: " << imu_data.acceleration().transpose() << endl;
    cout << "Position: " << position.transpose() << endl;
    odom_est.publish(ros::Time(gt_data->timestamp), position * POSITION_FACTOR, gt_data->attitude);

    odom_gt_real.publish(ros::Time(gt_data->timestamp), gt_data->position * POSITION_FACTOR, gt_data->attitude);
    odom_gt_T.publish(ros::Time(gt_T_data->timestamp), gt_T_data->position * POSITION_FACTOR, gt_T_data->attitude);
    
    cout << "dt: " << dt << endl;
    usleep(dt * 1000000U);
  }
  
}

int main_slam_euroc(ros::NodeHandle &n,int argc, char **argv) {
  
  // EuRoC config
  const uint SEED = 42;
  const string SEQ_PATH = "/media/javi/Datos/TFM/datasets/EuRoC/MH_05/files/";
  printf("Executing EuRoC test usign seq: %s\n", SEQ_PATH.c_str());

  EuRoC_Reader* euroc_reader = new EuRoC_Reader(SEQ_PATH);
  bool use_synthetic_acc = true;
  double dt = 0.0;
  double last_t = euroc_reader->current_img_data().timestamp;

  AccIntegrator pos_est(!use_synthetic_acc);


  // Synthetic acceleration config
  double mean = 0.0;
  double std = 0.0;
  if (mean != 0.0 or std != 0.0){
    euroc_reader->add_noise_to_synthetic_acc(mean, std, SEED);
  }
  
  
  // SLAM
  cv::Mat img;
  EuRoC_ImgData img_data;
  IMU_Measurements imu;

  // Read parameters
  SD_SLAM::Config &config = SD_SLAM::Config::GetInstance();
  if (!config.ReadParameters(argv[1])) {
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

  viewer = new SD_SLAM::Viewer(&SLAM, fdrawer, mdrawer);
  tviewer = new std::thread(&SD_SLAM::Viewer::Run, viewer);


  tracker->model_type = "imu_s";
  tracker->new_imu_model.set_remove_gravity_flag(false);
  
  Matrix4d identity_4d = Matrix4d::Identity();
  int count = 1; int max_count = euroc_reader->total_images();
  usleep(100000);
  ros::Rate r(30);

  while (ros::ok() && euroc_reader->has_next()) {
    printf("%d / %d\n", count, max_count); count++;

    euroc_reader->next();
    img_data = euroc_reader->current_img_data();
    imu = euroc_reader->current_imu_data();
    img = cv::imread(img_data.filename, -1);
    dt = img_data.timestamp - last_t; last_t = img_data.timestamp;

    if(img.empty()) {
      cout << "Img size: " << img.size() << endl;
      cerr << endl << "[ERROR] Failed to load image at: "  << img_data.filename << endl;
      ros::shutdown();
      return 1;
    }

    // ------ SLAM
    SD_SLAM::Timer ttracking(true);
    // Pass the image and measurements to the SLAM system
    Eigen::Matrix4d pose = SLAM.TrackNewFusion(img, imu, dt, identity_4d, img_data.timestamp); 

    // Set data to UI    
    fdrawer->Update(img, pose, tracker);
    mdrawer->SetCurrentCameraPose(pose, tracker->used_imu_model);


    ttracking.Stop();
    double delay = ttracking.GetTime();

    // Wait to load the next frame
    if(delay<dt)
      usleep((dt-delay)*1e6);

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Tests");
  ros::start();
  ros::NodeHandle n;    

  // TESTS
  //test_kitti_imu(n, 0);
  //test_euroc(n, 0);
  return main_slam_euroc(n, argc, argv);
  // end

  ros::shutdown();
  return 0;
}
