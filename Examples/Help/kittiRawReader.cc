
#include "kittiRawReader.h"
#include <iomanip> 

KittiRawReader::KittiRawReader(const std::string & dataset_path, 
                               const std::string & sequence):
                               dataset_path_(dataset_path)
{
  if ( RAW_SEQUENCES_MAP.find(sequence) == RAW_SEQUENCES_MAP.end() ) {
    std::cout << "Sequence not known." << std::endl;
    throw std::exception();
  } 
  sequence_ = RAW_SEQUENCES_MAP.at(sequence);
}
  

std::string KittiRawReader::_date_path(){
  std::string date = sequence_.date_drive.substr(0,10); 
  return this->dataset_path_ + "/" + date + "/";
}


std::string KittiRawReader::_sequence_path(){
  std::string date_path = this->_date_path();
  return date_path + sequence_.date_drive + "/";
}


std::vector<std::string> KittiRawReader::_read_filenames(std::string subfolder,
                                                         std::string file_extension){
  int start = this->sequence_.start;
  int end   = this->sequence_.end;
  int n_elements = end - start;

  std::vector<std::string> filenames;
  filenames.resize(n_elements);

  std::string prefix = this->_sequence_path() + subfolder;

  for(int i=start; i<start + n_elements; i++){
      std::stringstream ss;
      ss << std::setfill('0') << std::setw(10) << i;
      filenames[i] = prefix + ss.str() + file_extension;
  } 
  return filenames;
}

// TODO: De momento se emplea la frecuencia de captura (cte) de 10 Hz
std::vector<double> KittiRawReader::load_timestamps(){
  int start = this->sequence_.start;
  int end   = this->sequence_.end;
  int n_elements = end - start;

  std::vector<double> vec_timestamp;
  vec_timestamp.resize(n_elements);
  double dt = 0.1;  // 10 Hz

  for(int i=start; i<start + n_elements; i++){
    vec_timestamp[i] = i * dt;
  } 
  return vec_timestamp;
}


std::vector<std::string> KittiRawReader::load_left_images(){
  return this->_read_filenames("image_00/data/", ".png");
}


std::vector<std::string> KittiRawReader::load_right_images(){
  return this->_read_filenames("image_01/data/", ".png");
}


std::vector<std::string> KittiRawReader::load_velo(){
  return this->_read_filenames("oxts/data/", ".txt");
}

std::vector<std::vector<double>> KittiRawReader::load_groundtruth(){
  std::string filename = this->_sequence_path() + "gt.txt";

  std::vector<std::vector<double>> content;
  if (not this->_file_exists(filename)){
    std::cout << "[ERROR] GroundTruth file does not exists." << std::endl;
    return content;
  }

  std::ifstream file;
  file.open(filename.c_str());
  while(!file.eof()) {
    std::string line;
    std::getline(file, line);
    std::stringstream  line_stream(line);

    double value;
    std::vector<double> line_content;
    while(line_stream >> value){
      line_content.push_back(value);
    }
    content.push_back(line_content);
  }
  
  return content;
}


Eigen::Matrix3d KittiRawReader::load_imu_to_cam_matrix(){
  // TODO
  // Matrix3d imu_to_velo ( read_matrix3d(file, line) );
  // Matrix3d velo_to_cam  ( read_matrix3d(file, line) );
  // return imu_to_velo * velo_to_cam;
}


VeloSensor KittiRawReader::get_velo_data(const double timestamp, 
                                         const std::string & filename){
  if (not this->_file_exists(filename)){
    std::cout << "Velo file does not exists. Filepath: " + filename << std::endl;
    throw std::exception();
  }

  std::vector<double> content;
  double value;

  std::ifstream file;
  std::string line;
  file.open(filename.c_str());

  while(std::getline(file, line)) {
    std::stringstream  line_stream(line);
    while(line_stream >> value){
      content.push_back(value);
    }
  }
  file.close();

  VeloSensor data = {
    timestamp,
    {content[0], content[1], content[2]},     // oxts_units
    {content[3], content[4], content[5]},     // euler_angles
    {content[11], content[12], content[13]},  // acc_vehicle
    {content[14], content[15], content[16]},  // acc_sensor
    {content[17], content[18], content[19]},  // angular_vel_vehicle
    {content[20], content[21], content[22]}   // angular_vel_sensor
  };

  return data;
}


IMU_Measurements KittiRawReader::get_imu(const double timestamp, 
                                         const std::string & filename){

  VeloSensor velo = this->get_velo_data(timestamp, filename);
  Eigen::Vector3d acc(velo.acc_sensor[0], velo.acc_sensor[1], velo.acc_sensor[2]); 
  Eigen::Vector3d gyr(velo.angular_vel_sensor[0], velo.angular_vel_sensor[1], velo.angular_vel_sensor[2]);

  return IMU_Measurements(timestamp, acc, gyr);
}


cv::Mat KittiRawReader::get_image(const std::string & filename){
  if (not this->_file_exists(filename)){
    std::cout << "Image file not found. path: " + filename << std::endl;
    throw std::exception();
  }
  
  return cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
}