#pragma once

#include <vector>
#include "point.hpp"

/**
 * Interpolation of Lidar data.  Uses linear interpolation.  When data
 * is "missing" on either side of an interpolated point, the
 * interpolated point is considered missing as well (distance will be
 * zero).  This handles as gracefully as possible the case where the
 * Lidar returns a "low quality" data point or indicates that the
 * object at a specified angle was too close to measure the distance.
 *
 * @param points -- a vector of points assumed to be taken from Lidar.
 * You should include points with zero distance to replace "low
 * quality" points in the Lidar output.  See readLidar() in
 * lidar_support.  points must be in ascending order of angle.
 *
 * @param output -- output array of integer distances interpolated as
 * specified by the following two parameters.  You must allocate this
 * array to accomodate at least n elements.
 *
 * @param n -- size of output array (number of elements to be
 * produced)
 *
 * @param delta -- spacing, in degrees, between each angle in the
 * output.  output[0] will correspond to 0 degrees, output[1] will
 * correspond to delta degrees, output[m] (0 <= m < n)will correspond
 * to m*delta degrees.
 */

void interpolate(const std::vector<point>& points, int output[],
		 std::size_t n, double delta);

