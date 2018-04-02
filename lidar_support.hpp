#pragma once

#include "rplidar.h"
#include "point.hpp"

rp::standalone::rplidar::RPlidarDriver *setupLidar();

/**
 * read data from one scan (points have point.r and point.theta)
 *
 * if zeroLowQuality is true, set distance for low-quality
 * measurements to zero, otherwise discard them.
 */
vector<point> readLidar(rp::standalone::rplidar::RPlidarDriver * drv, bool zeroLowQuality);

/**
 * Take a vector of points and return a vector of 360 points filling
 * gaps in input vector with repeats.
 */
vector<point> always360(const vector<point>& inPoints);

