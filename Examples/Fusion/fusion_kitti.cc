

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

using namespace std;

void load_filenames(const string &basepath, vector<string> &vstrImageFilenames, vector<string> &vstrIMUFilenames);
IMU_Measurements load_IMU_data(const string &filename);

int main(int argc, char **argv)
{
    
  
  if(argc != 3) {
    cerr << endl << "Usage: ./fusion_kitti path_to_settings path_to_RAW_sequence" << endl;
    return 1;
  }

  // Read parameters
  SD_SLAM::Config &config = SD_SLAM::Config::GetInstance();
  if (!config.ReadParameters(argv[1])) {
    cerr << "[ERROR] Config file contains errors" << endl;
    return 1;
  }

  vector<string> images_filenames;
  vector<string> imu_filenames;
  load_filenames(string(argv[2]), images_filenames, imu_filenames);
  int n_images = images_filenames.size();

  if (n_images == 0) {
    cerr << "[ERROR] Couldn't find images. Check path." << endl;
    return 1;
  }
  cout << "[INFO] Sequence has " << n_images << " images" << endl;


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
  

  double freq = 1.0/10.0;
  int i = 0;
  cv::Mat img;
  IMU_Measurements imu;

  while (i < n_images && !SLAM.StopRequested()) {
    cout << "[INFO] Reading data " << i << "/" << n_images << endl;
    img = cv::imread(images_filenames[i], CV_LOAD_IMAGE_GRAYSCALE);
    imu = load_IMU_data(imu_filenames[i]);

    if(img.empty()) {
      cerr << endl << "[ERROR] Failed to load image at: "  << images_filenames[i] << endl;
      return 1;
    }

    SD_SLAM::Timer ttracking(true);

    // Pass the image and measurements to the SLAM system
    Eigen::Matrix4d pose = SLAM.TrackNewFusion(img, imu, freq);

    // Set data to UI
    fdrawer->Update(img, pose, tracker);
    mdrawer->SetCurrentCameraPose(pose);


    ttracking.Stop();
    double delay = ttracking.GetTime();

    // Wait to load the next frame
    if(delay<freq)
      usleep((freq-delay)*1e6);

    if (viewer->isFinished())
      return 0;

    i++;
  }

  // Stop all threads
  SLAM.Shutdown();

  viewer->RequestFinish();
  while (!viewer->isFinished())
    usleep(5000);
  tviewer->join();

  return 0;

}

inline bool file_exists (const std::string& name) {
    ifstream f(name.c_str());
    return f.good();
}

void load_filenames(const string &basepath, vector<string> &vstrImageFilenames, vector<string> &vstrIMUFilenames){

  string prefix_image = basepath + "/image_00/data/";  // left gray images
  string prefix_imu   = basepath + "/oxts/data/";

  int i = 0;
  while (true) {
    stringstream ss;
    ss << setfill('0') << setw(10) << i;

    string img_filename = prefix_image + ss.str() + ".png";
    string imu_filename = prefix_imu   + ss.str() + ".txt";

    if (! file_exists(img_filename) || !file_exists(imu_filename)){
      break;
    }
    vstrImageFilenames.push_back(img_filename);
    vstrIMUFilenames.push_back(imu_filename);
    i++;
  }

}

IMU_Measurements load_IMU_data(const string &filename){
  std::vector<double> content;
  ifstream file;
  file.open(filename.c_str());
  while(!file.eof()) {
    string line;
    getline(file, line);
    std::stringstream  line_stream(line);

    double value;
    while(line_stream >> value){
      content.push_back(value);
    }
  }
    
  Vector3d acceleration(content.at(11), content.at(12), content.at(13));
  Vector3d gyroscope   (content.at(17), content.at(18), content.at(19));
  IMU_Measurements imu_data = IMU_Measurements(0.0, acceleration, gyroscope);
  return imu_data;
}


