
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "inertial/IMU_Measurements.h"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Vector3d)

using namespace std;
using namespace Eigen;

struct EuRoC_ImgData{
  double timestamp;
  string filename;
};

struct EuRoC_IMUData{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp;
  Vector3d gyroscope;
  Vector3d accelerometer;
};

struct EuRoC_GTData{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp;
  Vector3d position;
  Quaterniond attitude;
  Vector3d velocity;
  Vector3d gyr_bias;
  Vector3d acc_bias;
};

class EuRoC_Reader{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Matrix3d rotation_imu_to_nwu();

  EuRoC_Reader(string sequence_path);
  void add_noise_to_synthetic_acc(const double & mean, const double & std, const uint SEED=42);


  bool has_next();
  bool next();
  inline EuRoC_GTData* current_gt_data(){return _get_gt_element(_gt_idx);}
  inline EuRoC_GTData* current_gt_origin_data(){return _get_gt_T_element(_gt_idx);}
  EuRoC_ImgData current_img_data();
  IMU_Measurements current_imu_data(bool use_synthetic_acc=true);

  inline uint total_images(){return _img_data.size();}

 private:
  Matrix3d R_imu_to_nwu;

  // Patths
  string _path;
  const string GT_FILE = "state_groundtruth_estimate0/data.csv";
  const string IMAGES_FOLDER = "cam0/data/";
  const string IMGS_FILE = "cam0/data.csv";
  const string IMU_FILE = "imu0/data.csv";

  // full data containers
  vector<EuRoC_GTData*> _gt_data;
  vector<EuRoC_GTData*> _gt_data_origin;
  vector<Vector3d> _acc_synthetic;
  vector<EuRoC_IMUData*> _imu_data;
  vector<EuRoC_ImgData> _img_data;

  // Current indexes 
  uint _gt_idx;  // valid for _gt_data, _gt_data_origin and _acc_synthetic
  uint _cam_idx;
  uint _imu_idx;




  // Update all indexes synchronously
  uint _next_sync_gt(double & timestamp);
  uint _next_sync_imu(double & timestamp);


  vector<EuRoC_GTData*> _read_gt_file(string sequence_path);
  vector<EuRoC_IMUData*> _read_imu_file(string sequence_path);
  vector<EuRoC_ImgData> _read_image_file(string sequence_path);

  EuRoC_GTData* _get_gt_element(uint idx);
  EuRoC_GTData* _get_gt_T_element(uint idx);
  EuRoC_IMUData* _get_imu_element(uint idx);
  EuRoC_ImgData _get_image_element(uint idx);
  Vector3d _get_synt_acc_element(uint idx);

  EuRoC_IMUData* _select_imu(double timestamp);
  Vector3d _select_synthetic_acc(double timestamp);

  vector<EuRoC_GTData*> _transform_gt_to_origin(const vector<EuRoC_GTData*> & gt);
  EuRoC_GTData* _select_gt(double timestamp);
  inline bool _file_exists (const std::string& name) {ifstream f(name.c_str()); return f.good();}

  void _generate_ideal_acc(vector<EuRoC_GTData*> & data);

};