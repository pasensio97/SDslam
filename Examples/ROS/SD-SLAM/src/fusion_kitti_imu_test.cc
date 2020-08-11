

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
#include "src/tests/kitti_help.h"
#include <random>
#include <Eigen/StdVector>
#include "src/inertial/tools/Estimator.h"

using namespace std;
using namespace Eigen;

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Vector3d)

const int SEED = 42;

void load_filenames(const string &basepath, vector<string> &vstrImageFilenames, vector<string> &vstrIMUFilenames);
IMU_Measurements load_IMU_data(const string &filename, double &time);
vector<vector<double>> load_gt_data(const string &filename);
inline bool file_exists (const std::string& name) {ifstream f(name.c_str()); return f.good();}
void create_file(const string & filename, const string & header="");
void write_line(const string & filename, const VectorXd & data, const int precision, string separator=",");
void set_rot_trans_from_gt(const vector<double> & rt, Quaterniond & rotation, Vector3d & translation);
Quaterniond cam_to_world(const Quaterniond & cam_attitude);
Vector3d quat_to_euler(const Quaterniond & q, bool to_degrees=true);
double rad_to_deg(double & angle);
Matrix4d vector_gt_to_pose(const vector<double> & rt);

vector<Vector3d> read_file_3_comps(string & filename){
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

vector<Vector3d> correct_synt_acc(const vector<Vector3d> & acc){
  vector<Vector3d> new_acc(acc.size());
  new_acc[0] = acc[0];
  for (int i=1; i<acc.size(); i++){
    new_acc[i] = new_acc[i-1] + acc[i];
  }
  return new_acc;
}

void write_it_info(const string &filename, int it, SD_SLAM::System &slam_system){
  string SEPARATOR = ",";

  std::fstream outfile;
	outfile.open(filename, std::fstream::app);

  outfile << it << SEPARATOR 
          << slam_system.GetTrackingState() << SEPARATOR 
          << slam_system.GetTracker()->stay_in_curve << SEPARATOR
          << slam_system.GetTracker()->first_proj << SEPARATOR
          << slam_system.GetTracker()->second_proj << SEPARATOR
          << slam_system.GetTracker()->inliers_on_pred << SEPARATOR
          << slam_system.GetTracker()->inliers_on_localmap 
          << endl;

  outfile.close();
}


double correct_angle(double const & angle){
  double new_angle = 0;
  if (angle > 0){
    new_angle = min(angle, 180-angle);
  }else if(angle < 0) 
    new_angle = -min(-angle, angle+180); 
  return new_angle;
}



class Rotations{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Rotations(){}

  Matrix3d kitti_imu2velo(){
    Matrix3d rotation_imu_to_velo;
    rotation_imu_to_velo << 9.999976e-01, 7.553071e-04, -2.035826e-03, 
                         -7.854027e-04, 9.998898e-01, -1.482298e-02,
                          2.024406e-03, 1.482454e-02,  9.998881e-01;
    return rotation_imu_to_velo;
  }

  Matrix3d kitti_velo2cam(){
    Matrix3d rotation_velo_to_cam;
    rotation_velo_to_cam <<  7.967514e-03, -9.999679e-01, -8.462264e-04,
                            -2.771053e-03,  8.241710e-04, -9.999958e-01,
                             9.999644e-01,  7.969825e-03, -2.764397e-03;
    return rotation_velo_to_cam;
  }

  Matrix3d cam2enu(){
    Matrix3d rotation_cam_to_enu;
    rotation_cam_to_enu  << 1,  0, 0,
                            0,  0, 1,
                            0, -1, 0;
    return rotation_cam_to_enu;
  }

  Matrix3d cam2nwu(){
    Matrix3d rotation_cam_to_enu;
    rotation_cam_to_enu  << 0,  0, 1,
                           -1,  0, 0,
                            0, -1, 0;
    return rotation_cam_to_enu;
  }

  Matrix3d nwu2enu(){
    Matrix3d rotation_nwu_to_enu;
   rotation_nwu_to_enu  << 0,-1, 0,
                            1, 0, 0,
                            0, 0, 1;
    return rotation_nwu_to_enu;
  }

  Matrix3d nwu2ned(){
    Matrix3d rotation_nwu_to_ned;
    rotation_nwu_to_ned  << 1, 0, 0,
                            0,-1, 0,
                            0, 0,-1;
    return rotation_nwu_to_ned;
  }
};


/**
 * Test: Crear la orientacion proporcionando a Madgwick la información en 3 sistemas diferentes.
*/
class test_attitude{
 private: 
  const string BASEFRAME = "odom";
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Madgwick nwu_mad;
  Madgwick enu_mad;

  TF_Publisher pub_nwu_attitude;
  TF_Publisher pub_enu_attitude;
  TF_Publisher pub_enu_on_nwu_attitude;

  Rotations rotations;

  test_attitude(ros::NodeHandle n, double gain=0.1):
    nwu_mad(Madgwick(gain)), 
    enu_mad(Madgwick(gain)),
    pub_nwu_attitude(TF_Publisher(n, BASEFRAME, "world_NWU")),
    pub_enu_attitude(TF_Publisher(n, BASEFRAME, "world_ENU")),
    pub_enu_on_nwu_attitude(TF_Publisher(n, BASEFRAME, "world_ENU_on_NWU"))
  {}

  void update(const IMU_Measurements & imu_raw){
    double dt = 0.1;
    auto acc = imu_raw.acceleration();
    auto gyr = imu_raw.angular_velocity();

    nwu_mad.update(acc, gyr, dt);
    enu_mad.update(rotations.nwu2enu() * acc, gyr, dt);

    Vector3d origin(0,0,0);
    Quaterniond enu_on_nwu(rotations.nwu2enu().inverse() * enu_mad.get_orientation().toRotationMatrix() * rotations.nwu2enu());

    pub_nwu_attitude.publish(origin, nwu_mad.get_orientation());
    pub_enu_attitude.publish(origin, enu_mad.get_orientation());
    pub_enu_on_nwu_attitude.publish(origin, enu_on_nwu);
  }
};

class Test_Pose{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PositionEstimator position_est;
  Madgwick attitude_est;
  Imu_Publisher pub_raw_acc;
  Imu_Publisher pub_linear_acc;
  Imu_Publisher pub_velocity;
  Rotations rot;
  int it;
  //Imu_Publisher pub_linear_acc;

  Test_Pose(ros::NodeHandle n):
    position_est(PositionEstimator(0.2)),
    attitude_est(Madgwick(0.0085)),
    pub_raw_acc(Imu_Publisher(n, "odom", "raw_acc")),
    pub_linear_acc(Imu_Publisher(n, "odom", "acc")),
    pub_velocity(Imu_Publisher(n, "odom", "vel")),
    it(0)
  {}

  void update(IMU_Measurements & imu_data, double &  dt){
    Vector3d acc = rot.cam2nwu() * (rot.kitti_velo2cam() * (rot.kitti_imu2velo() * imu_data.acceleration()));
    Vector3d gyr = rot.cam2nwu() * (rot.kitti_velo2cam() * (rot.kitti_imu2velo() * imu_data.angular_velocity()));

    attitude_est.update(acc, gyr, dt);
    
    position_est.update(acc, attitude_est.get_orientation(), dt, true);

    pub_raw_acc.publish(acc, gyr, ros::Time(it));
    pub_linear_acc.publish(position_est.acceleration(), gyr, ros::Time(it));
    pub_velocity.publish(position_est.velocity(), gyr, ros::Time(it));

    it++;
  }


};

void print_quat(const Quaterniond & q, string intro){
  printf("%s: (x: %.3f, y: %.3f, z: %.3f, w: %.3f)\n", intro.c_str(), q.x(), q.y(), q.z(), q.w());
}

vector<double> create_imu_vector(IMU_Measurements imu_data){
  Rotations r;
  auto acc = r.kitti_velo2cam() * (r.kitti_imu2velo() * imu_data.acceleration());
  auto gyr = r.kitti_velo2cam() * (r.kitti_imu2velo() * imu_data.angular_velocity());

  vector<double> v_imu;
  v_imu.push_back(gyr[0]);
  v_imu.push_back(gyr[1]);
  v_imu.push_back(gyr[2]);
  v_imu.push_back(acc[0]);
  v_imu.push_back(acc[1]);
  v_imu.push_back(acc[2]);
  return v_imu;
}

class Test_model_9{
  TF_Publisher pub_sdslam_cam;
  TF_Publisher pub_sdslam_world;
  TF_Publisher pub_model_cam;
  TF_Publisher pub_model_world;

 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Test_model_9(ros::NodeHandle & n):
    pub_sdslam_cam(n, "odom", "sdslam_cam"),
    pub_sdslam_world(n,"odom", "sdslam_world"),
    pub_model_cam(n, "odom", "model_cam"),
    pub_model_world(n, "odom","model_world")
  {}

  void publish(SD_SLAM::Tracking* &tracker, Matrix4d pose_cam){
    // model world
    //Vector3d pos_model_world = tracker->imu_model.position;
    //Quaterniond att_model_world = Quaterniond(tracker->imu_model.attitude);
    //pub_model_world.publish(pos_model_world, att_model_world.inverse().normalized());

    // sdslam world
    Vector3d pos_sdslam_world = -pose_cam.block<3,3>(0,0).transpose() * pose_cam.block<3,1>(0,3);
    Quaterniond att_sdslam_world = Quaterniond(pose_cam.block<3,3>(0,0));
    pub_sdslam_world.publish(pos_sdslam_world, att_sdslam_world.inverse().normalized());

    // model cam
    //Vector3d pos_model_cam = tracker->imu_model.position_cam;
    //Quaterniond att_model_cam = Quaterniond(tracker->imu_model.attitude);
    //pub_model_cam.publish(pos_model_cam, att_model_cam.normalized());

    // sdslam cam
    Vector3d pos_sdslam_cam = pose_cam.block<3,1>(0,3);
    Quaterniond att_sdslam_cam = Quaterniond(pose_cam.block<3,3>(0,0));
    pub_sdslam_cam.publish(pos_sdslam_cam, att_sdslam_cam.normalized());

  }
};

class motion_model_test{
 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector3d vel, pos;
  motion_model_test(){
    vel.setZero();
    pos.setZero();
  }
  Vector3d update(const Vector3d & acc, double dt){
    vel = vel + acc * dt;
    pos = pos + vel * dt;
    //cout << "acc " << acc.transpose() << endl;
    // cout << "vel " << vel.transpose() << endl;
    // cout << "pos " << pos.transpose() << endl;
    return pos;
  }
};

class KittiSeqSelect{
 private:
  const string DATASETS_PATH = "/media/javi/Datos/TFM/datasets/kitti/raw/";

 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  string SEQ;
  bool is_valid;
  string image_path;
  string imu_path;
  string gt_path;
  pair<int, int> limits;

  KittiSeqSelect(string seq){
    is_valid = true;
    this->SEQ = seq;
    string path;
    printf("Trying to read '%s' sequence\n", seq.c_str());

    if (seq == "seq_00"){
      path = "2011_10_03/2011_10_03_drive_0027_sync/";
      limits = {0, 4540};
    }
    else if (seq == "seq_00a"){
      path = "2011_10_03/2011_10_03_drive_0027_sync/";
      limits = {0, 1655};
    }
    else if (seq == "seq_00b"){
      path = "2011_10_03/2011_10_03_drive_0027_sync/";
      limits = {0, 740};
    }
    else if (seq == "seq_01"){
      path = "2011_10_03/2011_10_03_drive_0042_sync/";
      limits = {15, 1170};
    }
    else if (seq == "seq_02"){
      path = "2011_10_03/2011_10_03_drive_0034_sync/";
      limits = {100, 4660};
    }
    else if (seq == "seq_05"){
      path = "2011_09_30/2011_09_30_drive_0018_sync/";
      limits = {0, 2760};
    }
    else if (seq == "seq_06"){
      path = "2011_09_30/2011_09_30_drive_0020_sync/";
      limits = {110, 1100};
    }
    else if (seq == "seq_07"){
      path = "2011_09_30/2011_09_30_drive_0027_sync/";
      limits = {150, 1100};
    }
    else if (seq == "seq_09"){
      path = "2011_09_30/2011_09_30_drive_0033_sync/";
      limits = {100, 1590};
    }

    else{
      is_valid = false;
    }

    image_path = DATASETS_PATH + path + "image_00/data/";
    imu_path = DATASETS_PATH + path + "oxts/data/";
    gt_path = DATASETS_PATH + path + "gt.txt";
    printf("IMAGE PATH: %s\n", image_path.c_str());
    printf("IMU PATH:   %s\n", imu_path.c_str());
    printf("GT PATH:    %s\n", gt_path.c_str());
  }
};

vector<Vector3d> create_synthetic_acc(const vector<vector<double>> & gt_content){
  Kitti kitti;
  Matrix3d R = kitti.rotation_imu_cam().inverse();
  double dt = 0.1;
  int size = gt_content.size()-1;

  vector<Vector3d> velocity(size);
  velocity[0].setZero();
  // Velocity
  for (int i=1; i<size; i++){
    Vector3d s0 = vector_gt_to_pose(gt_content[i-1]).block<3,1>(0,3);
    Vector3d s1 = vector_gt_to_pose(gt_content[i]).block<3,1>(0,3);
    velocity[i] = (s1 - s0) / dt;
  }
  // acc
  vector<Vector3d> acc(size);
  acc[0].setZero();
  // Velocity
  for (int i=1; i<size; i++){
    Vector3d v0 = velocity[i-1];
    Vector3d v1 = velocity[i];
    acc[i] = R* ((v1 - v0) / dt);
  }

  return acc;
}

void add_noise(vector<Vector3d> &data, double mean, double std, uint SEED=42){
  default_random_engine generator;
  generator.seed(SEED); 
  std::normal_distribution<double> distribution(mean, std);

  for (int i=0; i<data.size(); i++){
    double noise = distribution(generator);
    data[i] = data[i] + Vector3d(noise, noise, noise);
  }
}

int main(int argc, char **argv)
{
  /*
  rosrun SD-SLAM KITTI_TEST /media/javi/Datos/TFM/datasets/kitti/raw/seq_00/2011_10_03/2011_10_03_drive_0027_sync /media/javi/Datos/TFM/datasets/kitti/raw/seq_00/2011_10_03/2011_10_03_drive_0027_sync/gt.txt /home/javi/tfm/camera_params/kitti.yaml 
  
  */
  ros::init(argc, argv, "KITTI_TEST");
  ros::start();
  ros::NodeHandle n;    

  /* 
  if(argc !=3 ) {
    cerr << endl << "Usage: KITTI_TEST config_file sequence " << endl;
    return 1;
  }
  */

  string seq = string(argv[2]);
  KittiSeqSelect kitti_seq(seq);
  if (!kitti_seq.is_valid){
    printf("Kitti sequence '%s' is not avalible\n", kitti_seq.SEQ.c_str());
    return 1;
  }

  Kitti kitti;
  vector<string> images_filenames = kitti.load_image_filenames(kitti_seq.image_path);
  vector<string> imu_filenames = kitti.load_imu_filenames(kitti_seq.imu_path);


  Matrix4d _last_pose;
  /*
  bool use_syn_acc = false;
  vector<Vector3d> syn_acc;
  if (use_syn_acc){
    string FILE_SYN_ACC = "/home/javi/tfm/tests/simulate_imu/temp_files/synthetic_acc.csv";
    syn_acc = read_file_3_comps(FILE_SYN_ACC);
    //syn_acc = correct_synt_acc(syn_acc);
  }
  */
  /*
  vector<string> images_filenames;
  vector<string> imu_filenames;
  load_filenames(string(argv[1]), images_filenames, imu_filenames);
  int n_images = images_filenames.size();
  */
  int n_images = images_filenames.size();
  if (n_images == 0) {
    cerr << "[ERROR] Couldn't find images. Check path." << endl;
    return 1;
  }
  cout << "[INFO] Sequence has " << n_images << " images" << endl;
  
  /*
  if (!file_exists(argv[2])){
    cerr << endl << "GT file does not exists" << endl;
    return 1;
  }
  //vector<vector<double>> gt_content = load_gt_data(argv[2]);
  */
  vector<vector<double>> gt_content = load_gt_data(kitti_seq.gt_path);

  TF_Publisher pub_gt_att_world = TF_Publisher(n, "odom", "world_gt_att");
  TF_Publisher pub_gt_att_cam   = TF_Publisher(n, "odom", "cam_gt_att");
  Vector3d origin(0,0,0);

    // Use SDSLAM
  SD_SLAM::Config &config = SD_SLAM::Config::GetInstance();
  if (!config.ReadParameters(argv[1])) {
    cerr << "[ERROR] Config file contains errors" << endl;
    return 1;
  }

  
  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  bool use_loop_closing = true;
  SD_SLAM::System SLAM(SD_SLAM::System::MONOCULAR_IMU_NEW, use_loop_closing); 

  // Create user interface
  SD_SLAM::Map * map = SLAM.GetMap();
  SD_SLAM::Tracking * tracker = SLAM.GetTracker();

  SD_SLAM::FrameDrawer * fdrawer = new SD_SLAM::FrameDrawer(map);
  SD_SLAM::MapDrawer * mdrawer = new SD_SLAM::MapDrawer(map);

  SD_SLAM::Viewer* viewer = nullptr;
  std::thread* tviewer = nullptr;

  viewer = new SD_SLAM::Viewer(&SLAM, fdrawer, mdrawer);
  tviewer = new std::thread(&SD_SLAM::Viewer::Run, viewer);

  vector<Vector3d> acc_syn;
  bool use_syn_acc = false;
  if (argc > 3){
    cout << "\nCONFIG TEST" << endl;
    string slam_type = string(argv[3]);
    if (slam_type == "mono"){
      tracker->__model = 0;
      tracker->model_type = "mono";
    }
    else if (slam_type == "gps"){
      // Predict pose using GPS and if model lost, publish gps pose until reloc
      tracker->__model = 14;
    } 
    else if (slam_type == "imu"){
      // Predict pose using IMU (acc real) and if model lost, try to reinitialize the map
      tracker->__model = 14;

      tracker->model_type = "imu";
      tracker->new_imu_model.set_remove_gravity_flag(true);
      tracker->new_imu_model.set_rotation_imu_to_world(kitti.rotation_imu_cam());
    } 
    else if (slam_type == "imu_s"){
      // Predict pose using IMU (acc synthetic) and if model lost, publish imu pose until reloc
      double mean = 0.0;
      double std = (argc>4) ? atof(argv[4]) : 0;
      uint seed = 42;
      use_syn_acc = true;

      //string FILE_SYN_ACC = "/home/javi/tfm/tests/simulate_imu/temp_files/synthetic_acc.csv";
      //acc_syn = read_file_3_comps(FILE_SYN_ACC);
      
      cout << "- Creating synthetic acc... ";
      acc_syn= create_synthetic_acc(gt_content);
      cout << "DONE!" << endl;
      
      if (std != 0){
        printf("- Adding guaussian noise to acc: N(%.2f, %.2f)...", mean, std);
        add_noise(acc_syn, mean, std, seed);
        printf("Done!\n");
      }

      tracker->model_type = "imu_s";
      tracker->new_imu_model.set_remove_gravity_flag(false);
      tracker->new_imu_model.set_rotation_imu_to_world(kitti.rotation_imu_cam());
      
    } 
    else if (slam_type == "gps_reinit"){
      
    } 
    else{
      cerr << endl << "[ERROR] Type does not recognize: "  << slam_type << endl;
      return 1;
    }
    printf("- Selected SLAM type '%s'", slam_type.c_str());

  }

  /*
  if (argc == 5){
    int model = atoi(argv[3]);
    printf("Selected model '%i'", model);
    tracker->__model = model;
  }
  */
  Imu_Publisher pub_imu = Imu_Publisher(n, "odom", "/imu");
  Quaterniond gt_att; 
  Vector3d gt_pos, last_gt_pos;
  Vector3d euler_angles;
  test_attitude test_att = test_attitude(n, 0.01);
  Quaterniond last_gt, last_slam;

  // FILES
  string SEPARATOR = " ";
  const string BASE_PATH = "/home/javi/tfm/TEMP_FILES/";
  string gt_outfile = BASE_PATH + "gt.txt";
  string scale_gt_outfile = BASE_PATH + "scale_gt.txt";
  create_file(gt_outfile, "timestamp, sx, sy, sz, qx, qy, qz, qw");
  create_file(scale_gt_outfile, "timestamp, scale");

  //string enu_outfile = BASE_PATH + "attitude_mad_01_accfilt_world.csv";


  //string error_imu_file = BASE_PATH + "error_imu_filt_rot.csv";
  //create_file(error_imu_file, "it, angular_distance (degrees)");
  //Madgwick imu_att_estimator = Madgwick(0.01);

  //string error_gt_slam_file = BASE_PATH + "error_gt_visual_slam.csv";
  //create_file(error_gt_slam_file, "it, angular_distance (degrees)");


  // TESTS 
  /*
  TF_Publisher pub_slam_att_world = TF_Publisher(n, "odom", "world_slam_att");
  TF_Publisher pub_slam_att_cam   = TF_Publisher(n, "odom", "cam_slam_att");
  TF_Publisher pub_imu_att_world  = TF_Publisher(n, "odom", "world_imu_att");
  TF_Publisher pub_imu_att_cam  = TF_Publisher(n, "odom", "cam_imu_att");
  
  LowPassFilter lpf_acc = LowPassFilter(3, 0.7);
  LowPassFilter lpf_gyr = LowPassFilter(3, 0.7);

  VectorXd error(2); double diff;
  */
  Rotations rot;

  Test_Pose test_pose = Test_Pose(n);
  Odom_Publisher odompub_gt_world(n, "odom", "odom_world_gt");
  Odom_Publisher odompub_slam_world(n, "odom", "odom_world_slam");
  Odom_Publisher odompub_slam_cam(n, "odom", "odom_local_cam");
 

  //Odom_Publisher odompub_gt_local(n, "odom", "odom_gt_local");
  //Odom_Publisher odompub_slam_local(n, "odom", "odom_slam_local");

  Odom_Publisher odompub_gps_world(n, "odom", "odom_world_gps");
  Odom_Publisher odompub_imu_world(n, "odom", "odom_world_imu");
  Odom_Publisher odompub_gps_cam(n, "odom", "odom_local_gps");
  //Odom_Publisher odompub_ekf(n, "odom", "odom_local_ekf");
  Imu_Publisher acc_pub(n, "odom", "imu_data");

  Quaterniond last_att, curr_att;
  Matrix3d att_mat;
  Matrix4d gps_pose_i;

  Test_model_9 test_imu_model(n);
  motion_model_test mmodel; TF_Publisher pub_mmodel(n, "odom", "mmodel");

  //Kitti kitti;
  double freq = 1.0/10.0;
  int i = kitti_seq.limits.first; // 3600
  cv::Mat img;
  IMU_Measurements imu;
  Eigen::Matrix4d last_pose = Eigen::Matrix4d::Identity();
  Vector3d last_pose_gt;
  bool is_first_pose = true;

  double total_time = 0.0 + freq*i;
  bool set_velocity = (tracker->model_type == "imu" or kitti_seq.limits.first > 0);

  // --- MAIN LOOP ---
  //int limit = min(n_images, (int) gt_content.size()-1);
  while (i < kitti_seq.limits.second) {
  //while (i < kitti_seq.limits.first + 200) {
    cout << "\n[INFO] Reading data " << i << "/" << kitti_seq.limits.second << endl;
    img = cv::imread(images_filenames[i], CV_LOAD_IMAGE_GRAYSCALE);
    imu = load_IMU_data(imu_filenames[i], total_time);
    if (use_syn_acc){ 
      imu = IMU_Measurements(imu.timestamp(), acc_syn[i], imu.angular_velocity()); 
      cout << "[INFO] Syntheic acc: " << acc_syn[i].transpose() << endl;
    }
    acc_pub.publish(imu.acceleration(), imu.angular_velocity());

    if(img.empty()) {
      cerr << endl << "[ERROR] Failed to load image at: "  << images_filenames[i] << endl;
      return 1;
    }
    

    //if (i> 450 && i<=549){img.setTo(cv::Scalar(0));}  // curva buena
    //if (i> 140 && i<=145){img.setTo(cv::Scalar(0));}
    //if (i> 140 && i<=150){img.setTo(cv::Scalar(0));}
    //if (i> 150 && i<=170){img.setTo(cv::Scalar(0));}
    ////if (i> 250 && i<=255){img.setTo(cv::Scalar(0));}
    //if (i> 1535 && i<=1560){img.setTo(cv::Scalar(0));}
    //if (i> 250){img.setTo(cv::Scalar(0));} // IMU
    //if (i> 390  && i<=445){img.setTo(cv::Scalar(0));}  // tercera cruva seq00
    //if (i> 450 && i<=549){img.setTo(cv::Scalar(0));}  // curva buena
    //if (i> 750  && i<=760){img.setTo(cv::Scalar(0));}

    ros::Time time_it = ros::Time(i);
    
    // ------GET GT

    set_rot_trans_from_gt(gt_content.at(i), gt_att, gt_pos);
    cout << "GT pos " << gt_pos.transpose() << endl;
    
    // SET GPS VALUES INTO SD-SLAM (in World-slam CS)
    //Vector3d gt_pos_on_slam = -gt_att.inverse().toRotationMatrix() * gt_pos;
    gps_pose_i = vector_gt_to_pose(gt_content.at(i));
    
    

    //gps_pose_i.block<3,3>(0,0) = gt_att.inverse().toRotationMatrix(); //gt_att.inverse().toRotationMatrix();
    //gps_pose_i.block<3,1>(0,3) = rot.cam2nwu() * gt_pos; //(-gps_pose_i.block<3,3>(0,0) * gps_pose_i.block<3,1>(0,3));

    // ------ SLAM
    SD_SLAM::Timer ttracking(true);
    // Pass the image and measurements to the SLAM system
    Eigen::Matrix4d pose = SLAM.TrackNewFusion(img, imu, freq, gps_pose_i, total_time); 
    cout << "sucess return of slam" << endl;
    cout << "Updating drawer... ";
    // Set data to UI    
    fdrawer->Update(img, pose, tracker);
    cout << "[TEST] using imu? " << tracker->used_imu_model << endl;
    bool is_predicted_with_imu = tracker->used_imu_model;
    mdrawer->SetCurrentCameraPose(pose, is_predicted_with_imu);

    // if (set_velocity and kitti_seq.limits.first + 1 == i){
    //   tracker->new_imu_model.reset();
    //   tracker->new_imu_model.set_velocity(kitti.rotation_imu_cam().inverse() * ((gt_pos - last_gt_pos) / freq));
    // }

    // SAVE GT 
    VectorXd gt_vec(8);
    gt_vec << total_time, gt_pos.x(), gt_pos.y(), gt_pos.z(), gt_att.x(), gt_att.y(), gt_att.z(), gt_att.w();
    write_line(gt_outfile, gt_vec, 19, SEPARATOR);

    //SAVE GT Scale 
    if (tracker->GetState() != tracker->NOT_INITIALIZED){
      Vector3d curr_slam_position = -pose.block<3,3>(0,0).transpose()      *  pose.block<3,1>(0,3);
      Vector3d last_slam_position = -last_pose.block<3,3>(0,0).transpose() *  last_pose.block<3,1>(0,3);
      if (is_first_pose){
        is_first_pose = false;
        last_slam_position.setZero();
      }
     
      double gt_scale = Estimator::scale(last_pose_gt, gt_pos, last_slam_position, curr_slam_position);
      VectorXd scale_vec(2);
      scale_vec << total_time, gt_scale;
      write_line(scale_gt_outfile, scale_vec, 15, SEPARATOR);
    }
    last_pose = pose;
    last_pose_gt = gt_pos;
    // cout << "Done " << endl;
    // // ------ end SLAM

    // cout << "Doing random things... ";
    // Matrix3d ekf = SLAM.GetTracker()->GetCurrentFrame().GetRotation();
    // Quaterniond gps_att = SLAM.GetTracker()->_att_test_gps;
    // Vector3d gps_pos = SLAM.GetTracker()->_pos_test_gps;
    // cout << "Done " << endl;
    
    // // odompub_ekf.publish(time_it, SLAM.GetTracker()->GetCurrentFrame().GetPosition(), Quaterniond(ekf));
    // cout << "Publishing... ";
    // odompub_gt_world.publish(time_it, (gt_pos)/27, gt_att);

    // double rviz_scale = 0.5; // for rviz 
    // odompub_gps_cam.publish(time_it, gps_pos * rviz_scale, gps_att);
    // odompub_gps_world.publish(time_it, (-gps_att.toRotationMatrix().transpose() * gps_pos) * rviz_scale, gps_att.inverse());

    // odompub_slam_cam.publish(time_it, pose.block<3, 1>(0, 3) * rviz_scale, Quaterniond(pose.block<3, 3>(0, 0)));
    // odompub_slam_world.publish(time_it, (-pose.block<3, 3>(0, 0).transpose() * pose.block<3, 1>(0, 3)) * rviz_scale, 
    //                                     Quaterniond(pose.block<3, 3>(0, 0).transpose()));

    // // -----------------------------------------------------------------------------------------------------

    // if (string(argv[3]) == "imu_s"){
    //   odompub_imu_world.publish(time_it, 
    //                             tracker->new_imu_model.get_pose_world().block<3,1>(0,3)*rviz_scale, 
    //                             Quaterniond(tracker->new_imu_model.get_pose_world().block<3,3>(0,0)).normalized());
    //   //odompub_imu_world.publish(time_it, Vector3d(0,0,0), Quaterniond(tracker->new_imu_model.get_pose_world().block<3,3>(3,3)).normalized());
    // }

    // test_imu_model.publish(tracker, pose);
    // if (use_syn_acc && i>=100){
    //   Vector3d curr_pos = -pose.block<3,3>(0,0).transpose() * pose.block<3,1>(0,3); 
    //   Vector3d last_pos = -_last_pose.block<3,3>(0,0).transpose() * _last_pose.block<3,1>(0,3); 
    //   if (i==100){
    //     //cout << "\n\tLAST POS: " << last_pos.transpose() << endl;
    //     mmodel.pos = last_pos;
    //     mmodel.vel = (curr_pos - last_pos) / freq;
    //     //cout << "\n\POS model: " << mmodel.pos.transpose() << endl;
    //   }
    //   Vector3d pos_mmodel = mmodel.update(acc_syn[i]/100, freq);
    //   pub_mmodel.publish(pos_mmodel, gt_att);
    //   if (i%5 == 0){
    //     mmodel.pos = curr_pos;
    //     mmodel.vel = (curr_pos - last_pos) / (freq);
    //   }
    // }
    // _last_pose = pose; 
    // //odompub_gt_world.publish(time_it, gt_pos/100, gt_att);
    // /*
    // // 1) WORLD
    // odompub_gt_world.publish(time_it, rot.cam2nwu() *  (gt_pos/100), cam_to_world(gt_att));

    // Quaterniond slam_word_att = cam_to_world(Quaterniond(pose.block<3, 3>(0, 0).transpose()));
    // Vector3d slam_world_pos = (-pose.block<3, 3>(0, 0).transpose() * pose.block<3, 1>(0, 3))/2;
    // odompub_slam_world.publish(time_it, rot.cam2nwu() * slam_world_pos, slam_word_att);
    
    // // 2) local
    // //    2.1) SLAM in same coordinates of GT
    // odompub_gt_world.publish(time_it, gt_pos/100, gt_att);
    // Quaternion rot_inv = Quaterniond(pose.block<3, 3>(0, 0)).inverse();
    // Vector3d pos = pose.block<3, 1>(0, 3);
    // odompub_slam_world.publish(time_it, (-rot_inv.toRotationMatrix()*pos) / 2, rot_inv));

    // //    2.2) GT in same coordinates of SLAM
    // Quaterniond gt_att_inv = gt_att.inverse();
    // odompub_gt_world.publish(time_it, (-gt_att_inv.toRotationMatrix() * gt_pos)/100, gt_att_inv);
    // odompub_slam_world.publish(time_it, pose.block<3, 1>(0, 3) / 2, Quaterniond(pose.block<3, 3>(0, 0)));
    // */
    // //    2.2) GT in same coordinates of SLAM ON THE FUSION POINT
    // //    2.3) WORLD in same coordinates of SLAM ON THE FUSION POINT
    // // -----------------------------------------------------------------------------------------------------

    cout << "Done " << endl;

    // -----------------------------------------------------------------------------------------------------
    // Wait to load the next frame
    ttracking.Stop();
    double delay = ttracking.GetTime();

    // Wait to load the next frame
    if(delay<freq)
      usleep((freq-delay)*1e6);

    
    //usleep(2.5e5);
    if (viewer->isFinished()){
      return 0;
    }
    //if (i == 200) {break;}
    i++;
    total_time += freq;
  }

  // Stop all threads
  SLAM.Shutdown();

  string PATH_TO_SAVE = "/home/javi/tfm/TEMP_FILES/";
  SLAM.save_as_tum(PATH_TO_SAVE + "traj.txt");
  SLAM.save_scales(PATH_TO_SAVE + "scales.txt");
  SLAM.save_tracking_state(PATH_TO_SAVE + "tracking_state.txt");

  while (!viewer->isFinished())
    usleep(5000);
  viewer->RequestFinish();
  while (!viewer->isFinished())
    usleep(5000);
  tviewer->join();


  ros::shutdown();
  return 0;

}


// Functions
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
      cout << "[ERROR] Couldnt read one of these files:\n\t* " << img_filename << "\n\t* " << imu_filename << endl;
      break;
    }
    vstrImageFilenames.push_back(img_filename);
    vstrIMUFilenames.push_back(imu_filename);
    i++;
  }

}

IMU_Measurements load_IMU_data(const string &filename, double &time){
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
  //frame
  //Vector3d acceleration(content.at(14), content.at(15), content.at(16));
  //Vector3d gyroscope   (content.at(20), content.at(21), content.at(22));
  IMU_Measurements imu_data = IMU_Measurements(time, acceleration, gyroscope);
  return imu_data;
}

void create_file(const string & filename, const string & header){
  //string filename = "/home/javi/tfm/tests/matches/new.csv";
  if (file_exists(filename)){
    printf("Overwritting file '%s'", filename.c_str());
  }

  std::fstream outfile;
	outfile.open(filename, std::fstream::out);
  outfile << "#" << header << std::endl;
	outfile.close();
}

void write_line(const string & filename, const VectorXd & data, const int precision, string separator){
  std::fstream outfile;
	outfile.open(filename, std::fstream::app);

  outfile << setprecision(precision);
  for (int indx=0; indx < data.size()-1; indx++){
    outfile << data[indx] << separator;
  }
  outfile << data[data.size()-1] << endl;
  outfile.close();
}

vector<vector<double>> load_gt_data(const string &filename){
  vector<vector<double>> content;
  if (not file_exists(filename)){
    cout << "[ERROR] GT file doesnt exists.." << endl;
    return content;
  }

  ifstream file;
  file.open(filename.c_str());
  while(!file.eof()) {
    string line;
    getline(file, line);
    std::stringstream  line_stream(line);

    double value;
    vector<double> line_content;
    while(line_stream >> value){
      line_content.push_back(value);
    }
    content.push_back(line_content);
  }
  
  return content;
}


void set_rot_trans_from_gt(const vector<double> & rt, Quaterniond & rotation, Vector3d & translation){
  translation = Vector3d(rt.at(3), rt.at(7), rt.at(11));

  Matrix3d rot;
  rot << rt.at(0), rt.at(1), rt.at(2),
         rt.at(4), rt.at(5), rt.at(6),
         rt.at(8), rt.at(9), rt.at(10);
  rotation = Quaterniond(rot);
}


Matrix4d vector_gt_to_pose(const vector<double> & rt){
  Matrix4d matrix_rt;
  matrix_rt << rt.at(0), rt.at(1), rt.at(2), rt.at(3),
               rt.at(4), rt.at(5), rt.at(6), rt.at(7),
               rt.at(8), rt.at(9), rt.at(10),rt.at(11),
               0,        0,        0,        1;

  return matrix_rt;
}

Quaterniond cam_to_world(const Quaterniond & cam_attitude){
  Matrix3d C2W;
  C2W << 0, 0, 1,
        -1, 0, 0,
         0,-1, 0;

  Matrix3d world;
  world = C2W * cam_attitude.toRotationMatrix() * C2W.transpose();
  Quaterniond world_q(world);
  return world_q.normalized(); 
}

double rad_to_deg(double & angle) {
  return angle * 180.0 / M_PI;
}

Vector3d quat_to_euler(const Quaterniond & q, bool to_degrees){
  Vector3d euler = q.toRotationMatrix().eulerAngles(0,1,2);
  if (to_degrees){
    euler[0] = correct_angle(rad_to_deg(euler[0])); 
    euler[1] = correct_angle(rad_to_deg(euler[1])); 
    euler[2] = correct_angle(rad_to_deg(euler[2])); 
  }
  return euler;
}

