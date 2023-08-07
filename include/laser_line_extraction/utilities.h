#ifndef LINE_EXTRACTION_UTILITIES_H
#define LINE_EXTRACTION_UTILITIES_H

#include <vector>
#include <cmath>

namespace line_extraction
{

struct CachedData
{
  std::vector<unsigned int> indices;
  std::vector<double> bearings;
  std::vector<double> cos_bearings;
  std::vector<double> sin_bearings;
};

struct RangeData
{
  std::vector<double> ranges;
  std::vector<double> xs;
  std::vector<double> ys;
};

struct Params
{
  double bearing_var;
  double range_var;
  double least_sq_angle_thresh;
  double least_sq_radius_thresh;
  double max_line_gap;
  double min_line_length;
  double min_range;
  double max_range;
  double min_split_dist;
  double outlier_dist;
  unsigned int min_line_points;
};

struct PointParams
{
  std::vector<double> a;
  std::vector<double> ap;
  std::vector<double> app;
  std::vector<double> b;
  std::vector<double> bp;
  std::vector<double> bpp;
  std::vector<double> c;
  std::vector<double> s;
};

// Inlining this function will be faster
// and also get rid of multiple definitions
// error
inline double pi_to_pi(double angle)
{
  angle = fmod(angle, 2 * M_PI);
  if (angle >= M_PI)
    angle -= 2 * M_PI;
  return angle;
}

inline geometry_msgs::Quaternion theta_to_quat(const double angle)
{
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = sin(angle/2);
  q.w = cos(angle/2);
  return q;
}

inline std::vector<double> polar_to_cartesian(boost::array<double, 2> &start_pt,
                                              boost::array<double, 2> &end_pt,
                                              const double &angle)
{
  std::vector<double> res;
  res.push_back((start_pt[0] * cos(angle)));
  res.push_back((start_pt[1] * sin(angle)));
  return res;
}

} // namespace line_extraction

#endif
