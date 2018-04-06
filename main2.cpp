#include <cstdlib>
#include <fstream>
#include "Position.hpp"
#include "Velocities.hpp"
#include "algorithms.hpp"
#include "rplidar.h"
#include "RPLaser.hpp"
#include "lidar_support.hpp"
#include "interpolator.hpp"
#include "motor_hat.h"
#include <vector>

using namespace rp::standalone::rplidar;

// --------------------------------------------------------------------------------
// SLAM stuff
// --------------------------------------------------------------------------------

const unsigned int MAP_SIZE_PIXELS = 500;
const unsigned int MAP_SIZE_MM = 5000;
const unsigned int CONVFACT = MAP_SIZE_PIXELS/MAP_SIZE_MM;
motor_hat::motor_hat mh;
RPlidarDriver *drv = setupLidar();
RPLaser laser;


double mm_to_pixel(double pos)
{
	double convFact = MAP_SIZE_PIXELS*1.0/MAP_SIZE_MM;
	return MAP_SIZE_PIXELS - pos*convFact;
}

int coords2index(double x,  double y)
{    
    return y * MAP_SIZE_PIXELS + x;
}

void vector_into_array(vector<point> v, int data[], size_t n) {
  for(size_t i=0; i < n && i < v.size(); i++) {
    data[i] = v[i].r;
  }
}

void map_2_image(unsigned char* map, std::vector<Position> pos)
 {
  ofstream image("image.pgm");
  image << "P2" << endl << MAP_SIZE_PIXELS << " " << MAP_SIZE_PIXELS << " 255" << endl;    

  for(Position p: pos) {
    cout << p.x_mm << ", " << p.y_mm << ": " 
         << mm_to_pixel(p.x_mm) << ", " << mm_to_pixel(p.y_mm) << ": " 
         << coords2index(mm_to_pixel(p.y_mm), mm_to_pixel(p.x_mm)) << endl;
    map[coords2index(mm_to_pixel(p.y_mm), mm_to_pixel(p.x_mm))] = 0;
  }

  for (size_t y=0; y<MAP_SIZE_PIXELS; y++) {
    for (size_t x=MAP_SIZE_PIXELS; x>0; x--) {
	//image << (int)map[coords2index(y,x-1)] << " ";
	if(map[coords2index(y,x-1)] < 70)
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

//stops all motors
void stopMoving()
{
  mh.set_speed(2,0,1);
  mh.set_speed(3,0,1);
  mh.set_speed(1,0,1);
  mh.set_speed(0,0,1);
}
//1 for right, -1 for left
void rotate(int i, int spd)
{
  mh.set_speed(2,spd,-i);
  mh.set_speed(3,spd,-i);
  mh.set_speed(1,spd,-i);
  mh.set_speed(0,spd,-i);
}

//pass 1 to move north and -1 to move south
void moveNS(int i, int spd)
{
  stopMoving();
  mh.set_speed(3,spd,-i);
  mh.set_speed(2,spd,i);
}

//pass 1 to move east and -1 to move west
void moveEW(int i, int spd)
{
  stopMoving();
  mh.set_speed(1, spd,i);
  mh.set_speed(0, spd+5,-i);
}


int findMinAngle()
{
 int numMins[360]; 
  for(int i=0; i<360; i++)
  {
    numMins[i]=0;
  }
 for(int i=0; i < 10; i++) 
 {
    int lidarData[360];
    vector<point> points = readLidar(drv, true);
    interpolate(points, lidarData, 360, 1); 

  int min=1000000;
  int angle = 1000;
  for(int i=0; i<360; i++)
  {
     if(lidarData[i]<min&&lidarData[i]!=0)
     {
        min=lidarData[i];
	angle=i;
     }
  }
     numMins[angle]++;
  }
  int max = 0;
  int maxan=0;
  for(int i = 0; i<360; i++)
  {
    //cout << numMins[i];
    //cout << endl;
    if(numMins[i]>max)
    {
      max = numMins[i];
      maxan = i;
    }
  }
  return maxan;

}

//angle is the angle of our robot we want closest to wall
void adjust(int angle)
{
  int s;
  int currangle = findMinAngle();
  if(currangle>270||currangle<90)
  {
    s=-1;
  }
  else
  {
    s=1;
  }
  
  while(currangle-angle>5||currangle-angle<-5)
  {
    rotate(s,20);
    currangle = findMinAngle();
    stopMoving();
  }
  //stopMoving();
  
  int lidarData[360];
  vector<point> points = readLidar(drv, true);
  interpolate(points, lidarData, 360, 1);
  
  int ourdist = lidarData[angle]; 
  
  while(ourdist > 160)
  {
    int lidarData[360];
    vector<point> points = readLidar(drv, true);
    interpolate(points, lidarData, 360, 1); 
    
    ourdist = lidarData[angle]; 
    moveEW(1,50);
  }
  while(ourdist < 160)
  {
    int lidarData[360];
    vector<point> points = readLidar(drv, true);
    interpolate(points, lidarData, 360, 1); 
    
    ourdist = lidarData[angle]; 
    moveEW(-1,50);
  }
  stopMoving();
}

void goNorth()
{
  int lidarData[360];
  vector<point> points = readLidar(drv, true);
  interpolate(points, lidarData, 360, 1);
  
  while(lidarData[0] != 0 && lidarData[0] > 215)
  {
    moveNS(1,150);
    cout << lidarData[0] << endl;
    if(lidarData[90] != 0 && lidarData[90]<185)
    {
    	rotate(-1,20);
    	cout << "close right" << endl;
    	delay(1000);
    	stopMoving();
    	delay(1000);
    	while(lidarData[90] != 0 && lidarData[90]<215) {
    		cout << "Still right" << endl;
	    	moveEW(-1,50);
		points = readLidar(drv, true);
    		interpolate(points, lidarData, 360, 1);
    	}
    	cout << "Resolved" << endl;
    	stopMoving();
    	delay(500);
    	
    }
    else if(lidarData[270] != 0 && lidarData[270]<185)
    {
    	rotate(1,20);
    	cout << "close left" << endl;
    	delay(1000); 
    	moveEW(1,50);
    	delay(500);  
    	stopMoving();
    	while(lidarData[270] != 0 && lidarData[270]<215) {
    		cout << "Still left" << endl;
	    	moveEW(1,50);
		points = readLidar(drv, true);
    		interpolate(points, lidarData, 360, 1);
    	}	
    	    	cout << "Resolved" << endl;
    	stopMoving();
    	delay(500);
    }
    points = readLidar(drv, true);
    interpolate(points, lidarData, 360, 1);
    
  }
  stopMoving();
}

void goSouth()
{
  int lidarData[360];
  vector<point> points = readLidar(drv, true);
  interpolate(points, lidarData, 360, 1);
  
  int ourdist = lidarData[180]; 
  while(ourdist > 250)
  {
    moveNS(-1,150);
    
    int lidarData[360];
    vector<point> points = readLidar(drv, true);
    interpolate(points, lidarData, 360, 1);
    
    ourdist = lidarData[180]; 
  }
  stopMoving();
}

void goEast()
{
  int lidarData[360];
  vector<point> points = readLidar(drv, true);
  interpolate(points, lidarData, 360, 1);
  
  int ourdist = lidarData[90]; 
  while(ourdist > 250)
  {
    moveEW(1,150);
    
    int lidarData[360];
    vector<point> points = readLidar(drv, true);
    interpolate(points, lidarData, 360, 1);
    
    ourdist = lidarData[90]; 
  }
  stopMoving();
}

void goWest()
{
  int lidarData[360];
  vector<point> points = readLidar(drv, true);
  interpolate(points, lidarData, 360, 1);
  
  int ourdist = lidarData[270]; 
  while(ourdist > 250)
  {
    moveEW(-1,150);
    
    int lidarData[360];
    vector<point> points = readLidar(drv, true);
    interpolate(points, lidarData, 360, 1);
    
    ourdist = lidarData[270]; 
  }
  stopMoving();
}

int main() {

  RPLaser laser;


/* //testing the motors
  moveNS(1,150);
  delay(3000);
  moveNS(-1,150);
  delay(3000);
  moveEW(1,150);
  delay(3000);
  moveEW(-1,150);
  delay(3000);
  stopMoving();
*/

  // throw away first 10
  for(int i=0; i < 10; i++) 
  {
    readLidar(drv, true);
  }
  
  //adjust(90);
  while(true)
  {
    goNorth();
    //goWest();
    //goSouth();
    //goEast();
    //adjust(90);
  }


  
}
