
#include "inertial/position_estimators/GPS.h"

GPS::GPS()
{
  _position.setZero();
  _last_reading.setZero();
  _is_initialize = false;
}

Vector3d GPS::update(const double latitude, const double longitude, const double altitude)
{
  Vector3d data = lecture_to_meters(latitude, longitude, altitude);

  if (! _is_initialize){
    _is_initialize = true;
    _last_reading = data;
    return _position;
  }
  _position += data - _last_reading;
  _last_reading = data;
  return Vector3d(_position);
}

Vector3d GPS::lecture_to_meters(const double latitude, const double longitude, const double altitude)
{
  double lat = _to_radians(latitude);
  double lon = _to_radians(longitude);
  double radius = EARTH_RADIUS + altitude;
  
  Vector3d data(
    radius * cos(lat) * cos(lon),
    radius * cos(lat) * sin(lon),
    radius * sin(lat));

  return data;
}
