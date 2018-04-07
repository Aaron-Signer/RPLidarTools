#include <cstdlib>
#include <fstream>
#include <sstream>
#include "Position.hpp"
#include "Velocities.hpp"
#include "algorithms.hpp"
#include "rplidar.h"
#include "RPLaser.hpp"
#include "lidar_support.hpp"
#include "interpolator.hpp"
#include "motor_hat.h"
#include "serial_command.hpp"
#include <wiringPi.h>
#include <vector>

using namespace rp::standalone::rplidar;
motor_hat::motor_hat mh;
RPlidarDriver *drv = setupLidar();
RPLaser laser;
int lidarData[360];



// ------------------------------
// Arduino communication
// ------------------------------

int arduinoSerialPort; // initialized below

const int flameSensorCount = 5;
int flameData[flameSensorCount];
const char ARDUINO_COMMAND_FIND_FLAME = 'f';
const char ARDUINO_COMMAND_READ_SOUND = 's';

const int FLAME_FAR_LEFT = 0;
const int FLAME_MID_LEFT = 1;
const int FLAME_CENTER = 2;
const int FLAME_MID_RIGHT = 3;
const int FLAME_FAR_RIGHT = 4;
const int ROTATION_DELAY = 500;
const int CHANGE_DIR_THRESH = 215;
const int TRANS_SPEED = 150;
const int RIGHT_LEFT_ADJ = 200;
const int RIGHT_LEFT_ADJ_WINDOW = RIGHT_LEFT_ADJ+30;
const int FLAME_THRESH = 600;

bool readFlames(int f[]) {
  sendArduinoCommand(arduinoSerialPort, ARDUINO_COMMAND_FIND_FLAME);
  string response = readArduinoResponse(arduinoSerialPort);
  //cout << "Arduino response: " << response << endl;
  stringstream ss;
  ss << response;
  
  if (! (ss >> f[0] >> f[1] >> f[2] >> f[3] >> f[4]) )
    return false;
  return true;
}

bool readSound() {
  sendArduinoCommand(arduinoSerialPort, ARDUINO_COMMAND_READ_SOUND);
  string response = readArduinoResponse(arduinoSerialPort);
  cout << "Arduino response: " << response << endl;

  return (response == "Y");
 }

int rightDirection(int d) 
{
  return (d + 3) % 4;
}
int leftDirection(int d) 
{
  return (d + 1) % 4; 
}

// moving robot
void move(int i, int spd)
{
  if(i == -1)
  {
    mh.set_speed(2,0,1);
    mh.set_speed(3,0,1);
    mh.set_speed(1,0,1);
    mh.set_speed(0,0,1);
  }
  if(i == 0)
  {
    mh.set_speed(3,spd,-1);
    mh.set_speed(2,spd,1);
    mh.set_speed(1,0,1);
    mh.set_speed(0,0,1);
  }
  if(i == 2)
  {
    mh.set_speed(3,spd,1);
    mh.set_speed(2,spd,-1);
    mh.set_speed(1,0,1);
    mh.set_speed(0,0,1);
  }
  if(i == 1)
  {
    mh.set_speed(2,0,1);
    mh.set_speed(3,0,1);
    mh.set_speed(1, spd,-1);
    mh.set_speed(0, spd+5,1);
  }
  if(i == 3)
  {
    mh.set_speed(2,0,1);
    mh.set_speed(3,0,1);
    mh.set_speed(1, spd,1);
    mh.set_speed(0, spd+10,-1);
  }
}

//1 for right, -1 for left
void rotate(int i, int spd)
{
  mh.set_speed(2,spd,-i);
  mh.set_speed(3,spd,-i);
  mh.set_speed(1,spd,-i);
  mh.set_speed(0,spd,-i);
}

//update lidar info
void updateLid(int lid [])
{
  vector<point> points = readLidar(drv, true);
  interpolate(points, lid, 360, 1); 
}

//returns an angle given a direction
int dirToAng(int d)
{
  switch (d)
  {
  case 0:
    return 0;
  case 1:
    return 270;
  case 2:
    return 180;
  case 3:
    return 90;
  }
  return -1;
}

//return the dist given a direction and lidarData
int rangeAtDirection(int d, int lid [])
{
  return lid[dirToAng(d)];
}

// -----------------------------------------------
// Fan code
// -----------------------------------------------

void setupFan() {
  pinMode(12, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(12, LOW);
  digitalWrite(5, LOW);
}

void fanOn()
{
  digitalWrite(12, HIGH);
  digitalWrite(5, HIGH);
}

void fanOff()
{
  digitalWrite(12, LOW);
}

void putOutFlame(int value)
{
  fanOn();
  while(flameData[FLAME_CENTER]>value-200)
  {
    cout << value-200 << endl;
    cout << flameData[FLAME_CENTER] << endl;
    readFlames(flameData);
    rotate(-1,20);
    delay(1500);
    rotate(1,20);
    delay(1500);
  }
  fanOff();
  move(-1,1);
}

void centerFlame() 
{
  while(true)
  {
    readFlames(flameData);
    if(flameData[FLAME_CENTER]>flameData[FLAME_FAR_LEFT]&&flameData[FLAME_CENTER]>flameData[FLAME_FAR_RIGHT]&&
      flameData[FLAME_CENTER]>flameData[FLAME_MID_RIGHT]&&flameData[FLAME_CENTER]>flameData[FLAME_MID_LEFT])
    {
      delay(1000);
      move(-1,1);
      putOutFlame(flameData[FLAME_CENTER]);
      break;
    }
    if(flameData[FLAME_FAR_RIGHT]>flameData[FLAME_FAR_LEFT])
    {
      rotate(1,20);
      delay(1000);
    }
    else
    {
      rotate(-1,20);
      delay(1000);
    }
  }
}

int trueAngle(int angle)
{
  if(angle>=360)
  {
    return (angle-360);
  }
  else if (angle < 0)
  {
    return(angle+360);
  }
  else
  {
    return angle;
  }
}

bool foundFlame()
{
  readFlames(flameData);
  bool found = false;
  for(int i = 0; i<5; i++)
  {
    if(flameData[i]>FLAME_THRESH)
    {
      found = true;
    }
  }
  return found;
}
/*
void adjust(int direction)
{
  int minAngle;
  while(our angle isnt min angle)
  {
    rotate(rotDir(dirToAng(direction), minAngle),ROTAION_SPEED);
  }
  move(-1,0);
}
*/
int rotDir(int currAngle, int desAngle)
{
  int i = currAngle;
  int itr;
  while(i != desAngle)
  {
    i++;
    i = trueAngle(i);
  }
  if(itr>180)
  {
    return -1;
  }
  else
  {
    return 1;
  }
}

int go(int direction)
{
  while(rangeAtDirection(direction,lidarData)!=0 && rangeAtDirection(direction,lidarData)>CHANGE_DIR_THRESH)
  {
    updateLid(lidarData);
    move(direction, TRANS_SPEED);
    if(foundFlame())
    {
      centerFlame();
      return 2;
    }
    if(rangeAtDirection(rightDirection(direction),lidarData)!=0 && rangeAtDirection(rightDirection(direction),lidarData)<RIGHT_LEFT_ADJ)
    {
      while(rangeAtDirection(rightDirection(direction),lidarData)!=0 && rangeAtDirection(rightDirection(direction),lidarData)<RIGHT_LEFT_ADJ_WINDOW)
      {
        move(leftDirection(direction),TRANS_SPEED);
      }
    }
    else if(rangeAtDirection(leftDirection(direction),lidarData)!=0 && rangeAtDirection(leftDirection(direction),lidarData)<RIGHT_LEFT_ADJ)
    {
      while(rangeAtDirection(leftDirection(direction),lidarData)!=0 && rangeAtDirection(leftDirection(direction),lidarData)<RIGHT_LEFT_ADJ_WINDOW)
      {
        move(rightDirection(direction),TRANS_SPEED);
      }
    }
  }
    move(-1,0);
  //  adjust(direction);
    direction++;
    return direction;
}

int main()
{

  arduinoSerialPort=setupArduinoSerial();
  
  // throw away first 10
  for(int i=0; i < 10; i++) 
  {
    readLidar(drv, true);
  }
  
  cout << "hello" << endl;
  while(!readSound())
  {
    cout << "nothin yet" << endl;
  }
  
  cout << "heard it" << endl;
  
  
  int i = 0;
  while(true)
  {
    int i = go(i);
    i = i%4;
  }
}
