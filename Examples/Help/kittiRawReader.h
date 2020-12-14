
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <tuple>
#include <map>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "inertial/IMU_Measurements.h"


struct RawSequence{
  std::string date_drive;
  int start;
  int end;
};

/*
  Nr.     Sequence name     Start   End
  ---------------------------------------
  00: 2011_10_03_drive_0027 000000 004540
  01: 2011_10_03_drive_0042 000000 001100
  02: 2011_10_03_drive_0034 000000 004660
  03: 2011_09_26_drive_0067 000000 000800
  04: 2011_09_30_drive_0016 000000 000270
  05: 2011_09_30_drive_0018 000000 002760
  06: 2011_09_30_drive_0020 000000 001100
  07: 2011_09_30_drive_0027 000000 001100
  08: 2011_09_30_drive_0028 001100 005170
  09: 2011_09_30_drive_0033 000000 001590
  10: 2011_09_30_drive_0034 000000 001200
*/
const std::map<std::string, RawSequence> RAW_SEQUENCES_MAP = {
    {"00",  {"2011_10_03_drive_0027_sync", 0,    4540}},
    {"00a", {"2011_10_03_drive_0027_sync", 0,    1655}},
    {"01",  {"2011_10_03_drive_0042_sync", 0,    1100}},
    {"02",  {"2011_10_03_drive_0034_sync", 0,    4660}},
    {"03",  {"2011_09_26_drive_0067_sync", 0,    800 }},
    {"04",  {"2011_09_30_drive_0016_sync", 0,    270 }},
    {"05",  {"2011_09_30_drive_0018_sync", 0,    2760}},
    {"06",  {"2011_09_30_drive_0020_sync", 0,    1100}},
    {"07",  {"2011_09_30_drive_0027_sync", 0,    1100}},
    {"08",  {"2011_09_30_drive_0028_sync", 1100, 5170}},
    {"09",  {"2011_09_30_drive_0033_sync", 0,    1590}},
    {"10",  {"2011_09_30_drive_0034_sync", 0,    1200}},
};


struct VeloSensor{
  double timestamp;
  std::vector<double> oxts_units;
  std::vector<double> euler_angles;
  std::vector<double> acc_vehicle;
  std::vector<double> acc_sensor;
  std::vector<double> angular_vel_vehicle;
  std::vector<double> angular_vel_sensor;
};

class KittiRawReader{
 private:
  std::string dataset_path_;
  RawSequence sequence_;

  // Funciones
  std::string _date_path();
  std::string _sequence_path();
  std::vector<std::string> _read_filenames(std::string subfolder, 
                                           std::string file_extension); 
  inline bool _file_exists (const std::string& name) {
    std::ifstream f(name.c_str()); 
    return f.good();
  }
 
 public:
  KittiRawReader(const std::string & dataset_path, 
                 const std::string & sequence);

  std::vector<double> load_timestamps();
  std::vector<std::string> load_left_images(); 
  std::vector<std::string> load_right_images();
  std::vector<std::string> load_velo();
  std::vector<std::vector<double>> load_groundtruth();
  Eigen::Matrix3d load_imu_to_cam_matrix();

  cv::Mat get_image(const std::string & filename);
  VeloSensor get_velo_data(const double timestamp, 
                           const std::string & filename);                                                          
  IMU_Measurements get_imu(const double timestamp, 
                           const std::string & filename);  
};
