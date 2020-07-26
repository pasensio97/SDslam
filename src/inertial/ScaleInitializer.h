
#ifndef SD_SLAM_SCALEINITIALIZER_H_
#define SD_SLAM_SCALEINITIALIZER_H_


#include <iostream>
#include <vector>
#include "Frame.h"
#include "inertial/PredictionModels.h"

using namespace std;
using namespace SD_SLAM;

class ScaleInitializer{
 public:
  enum eModel{
    DIRECT_SINGLE = 0,
    DIRECT_MULTIPLE = 1,
    SYNC = 2,
  };

  ScaleInitializer(const eModel model, uint num_scales=1);
  void on_map_initialization(
    new_IMU_model & motion_model, 
    double first_scale,
    const Frame & first_frame,
    const Frame & second_frame);

  void update(new_IMU_model & motion_model, double mean_scale);
  inline eModel get_model(){return _model;}
  inline bool is_initialized(){return _is_initialized;}
  inline double get_initial_scale(){return _initial_scale;}


 private:
  eModel _model;
  bool _is_initialized;
  uint _necessary_scales;
  vector<double> _buffer;

  double _initial_scale;
  uint _delete_scales;
  
};

#endif // SD_SLAM_SCALEINITIALIZER_H_