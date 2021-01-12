#ifndef SD_SLAM_GPS_POSITION_H_
#define SD_SLAM_GPS_POSITION_H_

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

class GPS{
 private:
  const double EARTH_RADIUS = 6371000.0;  // on meters
  Vector3d _last_reading;
  Vector3d _position;
  bool _is_initialize;

  inline double _to_radians(double degrees){return degrees * 180.0 / M_PI;}

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  GPS();

  /**
   * Add new lecture of GPS.
   * 
   * @param latitude of the gps-sensor on degrees 
   * @param longitude of the gps-sensor on degrees
   * @param altitude of the gps-sensor on meters
   * @return position updated with new value
  */
  Vector3d update(const double latitude, const double longitude, const double altitude);

  /**
   * Return current position vector
   * 
   * @return position
  */
  inline Vector3d get_position(){return Vector3d(_position);};

  /**
   * Transform lecture of gps into position on earth in meters
   * 
   * @param latitude of the gps-sensor on degrees 
   * @param longitude of the gps-sensor on degrees
   * @param altitude of the gps-sensor on meters
   * @return position in meters
  */
  Vector3d lecture_to_meters(const double latitude, const double longitude, const double altitude);

};



#endif  // SD_SLAM_GPS_POSITION_H_