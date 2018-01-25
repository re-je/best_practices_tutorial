#include "academy_lib/colors.h"

#include <math.h>

#include <opencv2/opencv.hpp>

#include <angles/angles.h>


#include <iostream>

std_msgs::ColorRGBA academy::getColor(double value) {
  double degrees = angles::to_degrees(angles::normalize_angle_positive(value));
  cv::Mat3f hsv(cv::Vec3f(degrees, 1.0, 1.0));
  cv::Mat3f bgr;
  cvtColor(hsv, bgr, CV_HSV2BGR);

  std_msgs::ColorRGBA res;
  res.b = bgr(0)[0];
  res.g = bgr(0)[1];
  res.r = bgr(0)[2];
  res.a = 1.0;
  return res;
}
