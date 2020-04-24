


#ifndef SD_SLAM_TOOLS_FILTERS_H_
#define SD_SLAM_TOOLS_FILTERS_H_

#include <Eigen/Dense>
using namespace Eigen;

class LowPassFilter{
 private:
  double _alpha;
  VectorXd _last_measure;

 public:
  LowPassFilter(int vector_size, const double & alpha);
  VectorXd apply(const VectorXd & measure);
  inline void set_initial_measure(VectorXd measure){_last_measure = measure;}
};

#endif  // SD_SLAM_IMUMEASUREMENTS_H_