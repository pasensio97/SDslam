
#include "kitti_help.h"
#include <iomanip> 

// read timestamp file: https://github.com/ethz-asl/kitti_to_rosbag/blob/5dc4f90befa97f44beaabcb24e1bd3d4f337c2ba/kitti_to_rosbag/src/kitti_parser.cpp

void Kitti::read_gps(string & filename, vector<Vector3d> &pos, vector<Quaterniond> &att){
  if (! _file_exists(filename)){
    printf("File does not exists. Filename: %s\n", filename.c_str());
    return;
  }

  ifstream file;
  file.open(filename.c_str());

  string line;
  Matrix3d aux_rot_matrix;
  double value;

  while(getline(file, line)) {    
    std::stringstream  line_stream(line);
    
    vector<double> line_content;
    while(line_stream >> value){
      line_content.push_back(value);
    }
    
    aux_rot_matrix << line_content[0], line_content[1], line_content[2],
                      line_content[4], line_content[5], line_content[6],
                      line_content[8], line_content[9], line_content[10];

    Vector3d position_i(line_content[3], line_content[7], line_content[11]);
    Quaterniond attitude_i(aux_rot_matrix);
    
    pos.push_back(position_i);
    att.push_back(attitude_i);
  }
  file.close();
}

vector<string> Kitti::load_imu_filenames(const string & filepath){
  //string prefix_imu   = basepath + "/oxts/data/";
  vector<string> imu_filenames;

  int i = 0;
  while (true) {
    stringstream ss;
    ss << setfill('0') << setw(10) << i;
    i++;

    string imu_filename = filepath + ss.str() + ".txt";
    if (!_file_exists(imu_filename)){
      break;
    }
    imu_filenames.push_back(imu_filename);
  }
  return imu_filenames;
}

vector<string> Kitti::load_image_filenames(const string & filepath){
  //string prefix_imu   = basepath + "/image_00/data/";
  vector<string> filenames;

  int i = 0;
  while (true) {
    stringstream ss;
    ss << setfill('0') << setw(10) << i;
    i++;

    string filename = filepath + ss.str() + ".png";
    if (!_file_exists(filename)){
      break;
    }
    filenames.push_back(filename);
  }
  return filenames;
}



IMUSensor Kitti::read_imu_file(const string & filename){

  std::vector<double> content;
  double value;

  ifstream file;
  string line;
  file.open(filename.c_str());

  while(getline(file, line)) {
    std::stringstream  line_stream(line);

    double value;
    while(line_stream >> value){
      content.push_back(value);
    }
  }
  file.close();

  IMUSensor data;
  data.oxts_units = Vector3d(content[0], content[1], content[2]);
  data.euler_angles = Vector3d(content[3], content[4], content[5]);
  data.acc_vehicle = Vector3d(content[11], content[12], content[13]);
  data.acc_sensor = Vector3d(content[14], content[15], content[16]);
  data.angular_vel_vehicle = Vector3d(content[17], content[18], content[19]);
  data.angular_vel_sensor =  Vector3d(content[20], content[21], content[22]);

  return data;
}


Matrix3d Kitti::rotation_imu_velo(){
  Matrix3d rotation_imu_to_velo;
  rotation_imu_to_velo << 9.999976e-01, 7.553071e-04, -2.035826e-03, 
                         -7.854027e-04, 9.998898e-01, -1.482298e-02,
                          2.024406e-03, 1.482454e-02,  9.998881e-01;
  return rotation_imu_to_velo;
}

Matrix3d Kitti::rotation_velo_cam(){
  Matrix3d rotation_velo_to_cam;
  rotation_velo_to_cam <<  7.967514e-03, -9.999679e-01, -8.462264e-04,
                          -2.771053e-03,  8.241710e-04, -9.999958e-01,
                           9.999644e-01,  7.969825e-03, -2.764397e-03;
  return rotation_velo_to_cam;
}

Matrix3d Kitti::rotation_imu_cam(){
  return rotation_velo_cam() * rotation_imu_velo();
}
