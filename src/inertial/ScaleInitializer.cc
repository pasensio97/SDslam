
#include "inertial/ScaleInitializer.h"
#include "inertial/tools/Estimator.h"

ScaleInitializer::ScaleInitializer(const eModel model, uint num_scales):
  _model(model), _is_initialized(false), _necessary_scales(num_scales), _initial_scale(0.0) , _delete_scales(0)
{
  if (_model == SYNC_VEL)
    _necessary_scales += _delete_scales;
}

void ScaleInitializer::on_map_initialization(
  new_IMU_model & motion_model, double first_scale, 
  const Frame & first_frame, const Frame & second_frame)
{
  if (_model == DIRECT_SINGLE){
      motion_model.set_scale(first_scale);
      _initial_scale = first_scale;
      _is_initialized = true;
  }

  else if (_model == DIRECT_MULTIPLE){
      _buffer.push_back(first_scale);
  }

  else if (_model == SYNC_VEL){
      //Ignore scale
      double dt = second_frame.mTimestamp - first_frame.mTimestamp;
      motion_model.reset();
      motion_model.correct_pose(second_frame, first_frame, dt, false);
  }
  else if (_model == SYNC_ZERO){
      //Ignore scale, just reset
      motion_model.reset();
  }
}

void ScaleInitializer::update(new_IMU_model & motion_model, double mean_scale)
{
  if (_is_initialized or _model == DIRECT_SINGLE )
    return;
    
  _buffer.push_back(mean_scale);
  if (_buffer.size() >= _necessary_scales){
    if (_model == SYNC_VEL){
      assert(_buffer.size()>_delete_scales);
      for (uint i=0; i<_delete_scales; i++){
        _buffer.erase(_buffer.begin());
      }
    }
    
    double scale = Estimator::mean(_buffer);
    motion_model.set_scale(scale);
    _initial_scale = scale;
    _is_initialized = true;

    //Verbose
    cout << "\t[TEST] ESTIMATE SCALE IMU-VISION USING THIS BUFFER OF SCALES: \n";
    for (double v : _buffer){cout << "\t* " << v << endl;}
    cout << "\t[TEST] MEAN: " << scale << endl;
  }


}