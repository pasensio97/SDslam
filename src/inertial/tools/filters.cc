
#include "inertial/tools/filters.h"


/*
class HighPassFilter{
 private:
  double _alpha;
  VectorXd _last_measure;

 public:
  HighPassFilter(int vector_size){
    _alpha = 1;
    _last_measure = VectorXd(vector_size);
    _last_measure.setZero();
  }

  Vector3d apply(const VectorXd & measure, const double & cutt_off, const double & dt){
    double reduced_freq = cutt_off / dt;
    printf("Reduced freq: %.3f", reduced_freq);
    _alpha = (1-exp(-2 * M_PI * reduced_freq));
    printf("alpha: %.3f", _alpha);
    _last_measure += _alpha*(measure - _last_measure);
    return measure - _last_measure;
  }

};
*/

LowPassFilter::LowPassFilter(int vector_size, const double & alpha){
  _alpha = alpha;
  _last_measure = VectorXd(vector_size);
  _last_measure.setZero();
}


Eigen::VectorXd LowPassFilter::apply(const VectorXd & measure){
    _last_measure = _alpha*measure + (1-_alpha)* _last_measure;
    return _last_measure;
  }
