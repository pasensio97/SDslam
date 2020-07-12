
#include "euroc_reader.h"
#include <iomanip> 
#include <random>

// read timestamp file: https://github.com/ethz-asl/kitti_to_rosbag/blob/5dc4f90befa97f44beaabcb24e1bd3d4f337c2ba/kitti_to_rosbag/src/kitti_parser.cpp

EuRoC_Reader::EuRoC_Reader(string sequence_path):
  _gt_idx(0), _cam_idx(0), _imu_idx(0)
{
  R_imu_to_nwu << 0.33638, -0.01749,  0.94156, 
                 -0.02078, -0.99972, -0.01114, 
                  0.94150, -0.01582, -0.33665;
  R_imu_to_nwu = R_imu_to_nwu.transpose();

  _path = sequence_path;
  _img_data = _read_image_file(sequence_path + IMGS_FILE);
  _imu_data = _read_imu_file(sequence_path + IMU_FILE);

  _gt_data = _read_gt_file(sequence_path + GT_FILE);
  _gt_data_origin = _transform_gt_to_origin(_gt_data);
  _generate_ideal_acc(_gt_data);


  cout << "Images elements: " << _img_data.size() <<endl;
  cout << "IMU elements: " << _imu_data.size()<<endl;
  cout << "GT elements: " << _gt_data.size()<<endl;
  cout << "GT T elements: " << _gt_data_origin.size()<<endl;
  cout << "Synt acc elements: " << _acc_synthetic.size()<<endl;

}

vector<EuRoC_GTData*> EuRoC_Reader::_read_gt_file(string sequence_path)
{
  printf("* Reading GT... ");
  vector<EuRoC_GTData*> gt;

  if (!_file_exists(sequence_path)){
    printf("ERROR.\nEuRoC GT file does not exists. Filename: %s\n", sequence_path.c_str());
    return gt;
  }

  ifstream file;
  file.open(sequence_path.c_str());

  string line;
  string str_value;

  while(getline(file, line)) {   
    if (line[0] == '#')
      continue;

    std::stringstream  line_stream(line);
    
    vector<double> line_content;
    while(getline(line_stream, str_value, ',')){
      line_content.push_back(stod(str_value));
    }
    
    EuRoC_GTData *gt_entry = new EuRoC_GTData();
    gt_entry->timestamp =  line_content[0] * 1e-9;
    gt_entry->position = Vector3d(line_content[1], line_content[2], line_content[3]);
    gt_entry->attitude =  Quaterniond(line_content[4], line_content[5],  line_content[6], line_content[7]);
    gt_entry->velocity = Vector3d(line_content[8], line_content[9], line_content[10]);
    gt_entry->gyr_bias = Vector3d(line_content[11], line_content[12], line_content[13]);
    gt_entry->acc_bias = Vector3d(line_content[14], line_content[15], line_content[16]);

    gt.push_back(gt_entry);
  }
  file.close();
  printf("DONE!\n");
  return gt;
}


// TODO: ROtate imu data
vector<EuRoC_IMUData*> EuRoC_Reader::_read_imu_file(string sequence_path)
{
  printf("* Reading IMU... ");

  vector<EuRoC_IMUData*> imu;

  if (!_file_exists(sequence_path)){
    printf("ERROR.\nEuRoC IMU file does not exists. Filename: %s\n", sequence_path.c_str());
    return imu;
  }

  ifstream file;
  file.open(sequence_path.c_str());

  string line;
  string str_value;

  while(getline(file, line)) {   
    if (line[0] == '#')
      continue;

    std::stringstream  line_stream(line);
    
    vector<double> line_content;
    while(getline(line_stream, str_value, ',')){
      line_content.push_back(stod(str_value));
    }
    
    EuRoC_IMUData* imu_entry = new EuRoC_IMUData();
    imu_entry->timestamp = line_content[0] * 1e-9;
    imu_entry->gyroscope = R_imu_to_nwu *  Vector3d(line_content[1], line_content[2], line_content[3]);
    imu_entry->accelerometer = R_imu_to_nwu * Vector3d(line_content[4], line_content[5],  line_content[6]);

    imu.push_back(imu_entry);
  }
  file.close();
  printf("DONE!\n");
  return imu;
}


vector<EuRoC_ImgData> EuRoC_Reader::_read_image_file(string sequence_path)
{
  printf("* Reading Images filames... ");
  vector<EuRoC_ImgData> images_data;

  if (!_file_exists(sequence_path)){
    printf("ERROR.\nEuRoC Images file does not exists. Filename: %s\n", sequence_path.c_str());
    return images_data;
  }

  ifstream file;
  file.open(sequence_path.c_str());

  string line;
  string str_value;

  while(getline(file, line)) {   
    if (line[0] == '#')
      continue;

    std::stringstream  line_stream(line);
    
    vector<string> line_content;
    while(getline(line_stream, str_value, ',')){
      line_content.push_back(str_value);
    }
    
    EuRoC_ImgData image_entry;
    image_entry.timestamp = stod(line_content[0]) * 1e-9;
    image_entry.filename = line_content[1];

    images_data.push_back(image_entry);
  }
  file.close();
  printf("DONE!\n");
  return images_data;
}


bool EuRoC_Reader::has_next()
{
  if (_img_data.empty() || _gt_data.empty() || _imu_data.empty())
    return false;
  
  if (_cam_idx+1 >= _img_data.size() || _gt_idx+1 >= _gt_data.size() || _imu_idx+1 >= _imu_data.size())
    return false;
  return true;
}


bool EuRoC_Reader::next()
{
  if (!has_next())
    return false;

  _cam_idx++;
  double timestamp = _get_image_element(_cam_idx).timestamp;
  _gt_idx = _next_sync_gt(timestamp);
  _imu_idx = _next_sync_imu(timestamp);
  printf("New indexes: Cam(%d), Gt(%d), IMU(%d)\n", _cam_idx, _gt_idx, _imu_idx);
  return true;
}


uint EuRoC_Reader::_next_sync_gt(double & timestamp)
{
  uint size = _gt_data.size();
  uint idx = 0;
  while (idx < size && _get_gt_element(idx)->timestamp <= timestamp){ idx++; }
  
  if (idx>0)
    idx--;
 
  return idx; 
}


uint EuRoC_Reader::_next_sync_imu(double & timestamp)
{
  uint size = _imu_data.size();
  uint idx = 0;
  while (idx < size && _get_imu_element(idx)->timestamp <= timestamp){ idx++; }
  
  if (idx>0)
    idx--;
 
  return idx; 
}

EuRoC_ImgData EuRoC_Reader::current_img_data()
{
  EuRoC_ImgData img_data = _get_image_element(_cam_idx); 
  EuRoC_ImgData out_data;

  out_data.filename = _path + IMAGES_FOLDER + string(img_data.filename);
  out_data.timestamp = double(img_data.timestamp);

  return out_data;
}

IMU_Measurements EuRoC_Reader::current_imu_data(bool use_synthetic_acc)
{
  EuRoC_IMUData* imu_data = _get_imu_element(_imu_idx);
  double timestamp = imu_data->timestamp;
  Vector3d acc = (use_synthetic_acc) ? _get_synt_acc_element(_cam_idx) : imu_data->accelerometer;
  Vector3d gyr = imu_data->gyroscope;

  return IMU_Measurements(timestamp, acc, gyr);
}


vector<EuRoC_GTData*> EuRoC_Reader::_transform_gt_to_origin(const vector<EuRoC_GTData*> & gt)
{
  printf("* Transforming GT to torigin... ");
  vector<EuRoC_GTData*> gt_transformed;

  Quaterniond att_0_inv = gt[0]->attitude.inverse();
  Vector3d pos_0 = att_0_inv * gt[0]->position;


  for (uint idx=0; idx < gt.size(); idx++){
    EuRoC_GTData* gt_data = new EuRoC_GTData();

    Quaterniond new_att = gt[idx]->attitude * att_0_inv;

    gt_data->position = gt[idx]->position - gt[0]->position;
    gt_data->attitude = new_att.normalized();
    gt_transformed.push_back(gt_data);
  }
  printf("DONE\n");
  return gt_transformed;
}



EuRoC_GTData* EuRoC_Reader::_select_gt(const double timestamp)
{
  while(_gt_data[_gt_idx]->timestamp <= timestamp){
    _gt_idx++;
  }
  return (_gt_idx>0) ? _gt_data[_gt_idx - 1] : _gt_data[0];
}


void EuRoC_Reader::add_noise_to_synthetic_acc(const double & mean, const double & std, const uint SEED){
  printf("* Adding normal noise to acc ideal with %.3f mean and %.3f std... ", mean, std);
  default_random_engine generator;
  generator.seed(SEED); 
  std::normal_distribution<double> distribution(mean, std);

  for (int i=0; i<(int) _acc_synthetic.size(); i++){
    double noise = distribution(generator);
    _acc_synthetic[i] += Vector3d(noise, noise, noise);
  }
  printf("DONE!\n");
}

void EuRoC_Reader::_generate_ideal_acc(vector<EuRoC_GTData*> & gt_data)
{
  printf("* Generating ideal acc... ");
  // ideally we must to add gravity component
  
  // dt based on img info
  int n_images = total_images();
  vector<double> dt_vec(n_images, 0);
  for (int i=1; i<n_images; i++){
    dt_vec[i] = (_img_data[i].timestamp - _img_data[i-1].timestamp);
  }

  vector<EuRoC_GTData*> data;
  for (EuRoC_ImgData img_data : _img_data){
    data.push_back(_get_gt_element(_next_sync_gt(img_data.timestamp)));
  }

  // Velocity
  vector<Vector3d> velocity(n_images, Vector3d(0,0,0));
  for (int i=1; i<n_images; i++){
    velocity[i] = (data[i]->position - data[i-1]->position) / dt_vec[i];
  }

  // Acc
  _acc_synthetic = vector<Vector3d>(n_images, Vector3d(0,0,0));
  for (int i=1; i<n_images; i++){
    _acc_synthetic[i] = (velocity[i] - velocity[i-1]) / dt_vec[i];
  }
  printf("DONE!\n");
}


EuRoC_GTData* EuRoC_Reader::_get_gt_element(uint idx)
{
  if(idx < 0 || idx >= _gt_data.size())
    throw std::out_of_range("Index out of range of GT data");
  
  return _gt_data[idx];
}


EuRoC_GTData* EuRoC_Reader::_get_gt_T_element(uint idx)
{
  if(idx < 0 || idx >= _gt_data_origin.size())
    throw std::out_of_range("Index out of range of GT Transformed data");
  
  return _gt_data_origin[idx];
}



EuRoC_IMUData* EuRoC_Reader::_get_imu_element(uint idx)
{
  if(idx < 0 || idx >= _imu_data.size())
    throw std::out_of_range("Index out of range of IMU data");
  
  return _imu_data[idx];
}


EuRoC_ImgData EuRoC_Reader::_get_image_element(uint idx)
{
  if(idx < 0 || idx >= _img_data.size())
    throw std::out_of_range("Index out of range of IMG data");
  
  return _img_data[idx];
}

Vector3d EuRoC_Reader::_get_synt_acc_element(uint idx)
{
  if(idx < 0 || idx >= _acc_synthetic.size())
    throw std::out_of_range("Index out of range of acc synthetic data");
  
  return _acc_synthetic[idx];
}