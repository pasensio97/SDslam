/**
 *
 *  Copyright (C) 2020 Javier Martinez
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
#include "System.h"
#include "Tracking.h"
#include "Map.h"
#include "Config.h"
#include "extra/timer.h"
#include "ui/Viewer.h"
#include "ui/FrameDrawer.h"
#include "ui/MapDrawer.h"
#include "Converter.h"
#include "inertial/IMU_Measurements.h"
#include "Examples/Help/kittiRawReader.h"
#include <random>
#include <Eigen/StdVector>

using namespace std;
using namespace Eigen;

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Vector3d)


void ShowPose(const Eigen::Matrix4d &pose) {
  Eigen::Matrix4d wpose;
  wpose.setIdentity();
  wpose.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0).transpose();
  wpose.block<3, 1>(0, 3) = -pose.block<3, 3>(0, 0).transpose()*pose.block<3, 1>(0, 3);

  Eigen::Quaterniond q(wpose.block<3, 3>(0, 0));
  cout << "[INFO] World pose: [" << wpose(0, 3) << " " << wpose(1, 3) << " " << wpose(2, 3) << "]";
  cout << "[" << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "]" << endl;
}


vector<Vector3d> create_synthetic_acc(const vector<vector<double>> & gt_content,
                                      const Matrix3d & imu_to_cam){

  Matrix3d R_inv = imu_to_cam.inverse();
  double dt = 0.1;
  int size = gt_content.size()-1;

  vector<Vector3d> velocity(size);
  velocity[0].setZero();
  // Velocity
  for (int i=1; i<size; i++){
    Vector3d s0(gt_content[i-1][3], gt_content[i-1][7], gt_content[i-1][11]);
    Vector3d s1(gt_content[i][3], gt_content[i][7], gt_content[i][11]);
    velocity[i] = (s1 - s0) / dt;
  }

  // acc
  vector<Vector3d> acc(size);
  acc[0].setZero();
  for (int i=1; i<size; i++){
    Vector3d v0 = velocity[i-1];
    Vector3d v1 = velocity[i];
    Vector3d acc_i = ((v1 - v0) / dt);
    acc[i] = R_inv*acc_i;
  }

  return acc;
}

void add_noise(vector<Vector3d> &data, double mean, double std, uint seed=42){
  default_random_engine generator;
  generator.seed(seed); 
  std::normal_distribution<double> distribution(mean, std);
  int size = data.size();

  for (int i=0; i<size; i++){
    double noise = distribution(generator);
    data[i] = data[i] + Vector3d(noise, noise, noise);
  }
}

int main(int argc, char **argv) {
  if(argc != 6){
    cerr << endl << "Usage: ./mono_kitti dataset_path sequence config_yaml bias std" << endl;
    return 1;
  }

  string dataset_path = argv[1];
  string sequence = argv[2];
  double bias = atof(argv[4]);
  double std = atof(argv[5]);

  SD_SLAM::Config &config = SD_SLAM::Config::GetInstance();
  if (!config.ReadParameters(argv[3])) {
    cerr << "[ERROR] Config file contains errors" << endl;
    return 1;
  }


  std::cout << "Reader" << std::endl;
  KittiRawReader kitti(dataset_path, sequence);
  std::vector<double> timestamps = kitti.load_timestamps();
  std::vector<std::string> image_filenames = kitti.load_left_images();
  std::vector<std::string> velo_filenames = kitti.load_velo();
  std::vector<std::vector<double>> gt_data = kitti.load_groundtruth();

  uint n_images = image_filenames.size();

  if (gt_data.size() == n_images){
    cerr << "[ERROR] GT file must be the same size as n_images." << endl;
    return 1;
  }

  cout << "Creating synthetic acc... ";
  Matrix3d rot_imu_to_world = Converter::toMatrix3d(Config::RotationIMUToCam());
  vector<Vector3d> acc_syn;
  acc_syn = create_synthetic_acc(gt_data, rot_imu_to_world);
  double SEED = 42;
  add_noise(acc_syn, bias, std, SEED);
  cout << "Done" << endl;

  double dt = 0.1;
  uint ni = 0;

  IMU_Measurements imu;
  cv::Mat image;

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

  tracker->new_imu_model.set_remove_gravity_flag(false);
  
  // Main loop
  while (ni < n_images && !SLAM.StopRequested()) {
    printf("[%i/%i]\n", ni, n_images);

    image = kitti.get_image(image_filenames[ni]);
    imu = kitti.get_imu(timestamps[ni], velo_filenames[ni]);
    imu = IMU_Measurements(imu.timestamp(), acc_syn[ni], imu.angular_velocity()); 

    tracker->new_imu_model.set_remove_gravity_flag(false);


    // Ignore first frames to
    if (ni < 4) { image.setTo(cv::Scalar(0)); }
    // Force lost
    if (ni> 390  && ni<=445) { image.setTo(cv::Scalar(0)); }

    SD_SLAM::Timer ttracking(true);

    // Pass the image to the SLAM system
    Eigen::Matrix4d pose = SLAM.TrackNewFusion(image, imu, dt, timestamps[ni]);

    // Show world pose
    ShowPose(pose);

    // Set data to UI
    fdrawer->Update(image, pose, tracker);
    mdrawer->SetCurrentCameraPose(pose, tracker->used_imu_model);

    ttracking.Stop();
    double delay = ttracking.GetTime();

    // Wait to load the next frame
    if(delay < dt)
      usleep((dt-delay)*1e6);

    if (viewer->isFinished())
      return 0;

    ni++;
  }
  

  // Stop all threads
  SLAM.Shutdown();

  viewer->RequestFinish();
  while (!viewer->isFinished())
    usleep(5000);

  tviewer->join();

  return 0;
}
