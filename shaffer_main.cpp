#include <cstdlib>
#include <fstream>
#include "Position.hpp"
#include "Velocities.hpp"
#include "algorithms.hpp"
#include "rplidar.h"
#include "RPLaser.hpp"
#include "lidar_support.hpp"
#include "interpolator.hpp"

using namespace rp::standalone::rplidar;

// --------------------------------------------------------------------------------
// SLAM stuff
// --------------------------------------------------------------------------------

const unsigned int MAP_SIZE_PIXELS = 800;

int coords2index(double x,  double y)
{    
    return y * MAP_SIZE_PIXELS + x;
}

void vector_into_array(vector<point> v, int data[], size_t n) {
  for(size_t i=0; i < n && i < v.size(); i++) {
    data[i] = v[i].r;
  }
}

void map_2_image(unsigned char* map) {
  ofstream image("image.pgm");
  image << "P2" << endl << MAP_SIZE_PIXELS << " " << MAP_SIZE_PIXELS << " 255" << endl;    
  for (size_t y=0; y<MAP_SIZE_PIXELS; y++) {
    for (size_t x=MAP_SIZE_PIXELS; x>0; x--) {
	if(map[coords2index(y,x-1)] < 10)
      		image << 0 << " ";
	else
		image << 255 << " ";
    }
    image << endl;
  }
  image.close();
}

void print_map(unsigned char* map) {
  for(size_t row=0; row < MAP_SIZE_PIXELS; row++) {
    for(size_t col=0; col < MAP_SIZE_PIXELS; col++) {
      cout << (int)map[coords2index(col,row)] << " ";
      // if (map[coord2index(col,row)] != 0)
      // 	cout << "X";
      // else
      // 	cout << " ";
    }
    cout << endl;
  }
}

int main() {

  unsigned char * map = new unsigned char[MAP_SIZE_PIXELS * MAP_SIZE_PIXELS];
  RPLaser laser;
  RMHC_SLAM slam(laser, MAP_SIZE_PIXELS, 4, time(NULL));
  
  RPlidarDriver *drv = setupLidar();

  // throw away first 10
  for(int i=0; i < 10; i++) {
    readLidar(drv, true);
  }


  // OK, I know part of the problem now.  the function always360 is
  // broken.  Need to interpolate on the data set to get 360 data
  // points.
  
  for(int i=0; i < 100; i++) {
    int lidarData[360];
    vector<point> points = readLidar(drv, true);
    interpolate(points, lidarData, 360, 1);

    // for(point p: points) cout << p.r << " ";
    // cout << endl << "----------------" << endl;
     for(size_t i=0; i < 360; i++) 
	cout << i << " " << lidarData[i] << endl;

     //cout << endl << "------------------------------" << endl;

    
    slam.update(lidarData);

    Position p = slam.getpos();
    //cout << p.x_mm << " " << p.y_mm << endl;
  }
  
  slam.getmap(map);
  map_2_image(map);
  drv->stopMotor();
  
}
