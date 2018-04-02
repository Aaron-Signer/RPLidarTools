#include <cstdlib>
#include <fstream>
#include "Position.hpp"
#include "Velocities.hpp"
#include "algorithms.hpp"
#include "rplidar.h"
#include "RPLaser.hpp"
#include "lidar_support.hpp"
#include "motor_hat.h"

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
    for (size_t x=0; x<MAP_SIZE_PIXELS; x++) {
      image << (int)map[coords2index(x,y)] << " ";
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

vector<point> points;
  unsigned char * map = new unsigned char[MAP_SIZE_PIXELS * MAP_SIZE_PIXELS];
  RPLaser laser;
  
  RPlidarDriver *drv = setupLidar();

  for(int i=0; i < 1; i++) {
    int lidarData[720];
     points = readLidar(drv);
    vector_into_array(always720(points), lidarData, 720);
  }
	
for(point p:)
	cout << points[i];  

  drv->stopMotor();
  
}
