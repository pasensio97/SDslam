
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <ros/ros.h>
#include "inertial/IMU_Measurements.h"
#include "inertial/Madgwick.h"
#include "tests/kitti_help.h"
#include "tests/publishers.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

vector<Vector3d> read_file(string & filename){
  ifstream file;
  file.open(filename.c_str());

  string line;
  vector<Vector3d> content;
  double value;

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



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Tests");
  ros::start();
  ros::NodeHandle n;    

  // TESTS
  test_kitti_imu(n, 0);
  // end

  ros::shutdown();
  return 0;
}
