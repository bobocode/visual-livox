#ifndef _COMMON_H_
#define _COMMON_H_

#include <pcl/point_types.h>

#include <cmath>

typedef pcl::PointXYZI PointType;

inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}
inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

#endif