/*
 * Copyright (C) 2015 Ben Shaffer
 */

#include <cmath>
#include <vector>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <cassert>

//#include "line.h"
#include "point.hpp"
using namespace std;

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

const char* TTY = "/dev/ttyUSB0";


using namespace rp::standalone::rplidar;

typedef rplidar_response_measurement_node_t lidarNode;


bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        //cout << "RPLidar health status : " << (int)healthinfo.status << endl;
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
	    cerr << "Error, rplidar internal error detected. Please reboot the device to retry." << endl;
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
	cerr << "Error, cannot retrieve the lidar health code:" << op_result << endl;
        return false;
    }
}

RPlidarDriver *setupLidar(){
    const char * opt_com_path = NULL;
    _u32         opt_com_baudrate = 115200;
 
    opt_com_path = TTY;

    // create the driver instance
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }


    // make connection...
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
	RPlidarDriver::DisposeDriver(drv);
	exit(0);
    }

    // check health...
    if (!checkRPLIDARHealth(drv)) {
      RPlidarDriver::DisposeDriver(drv);
      exit(0);
    }


    rplidar_response_device_info_t devinfo;

    u_result     op_result;
    op_result = drv->getDeviceInfo(devinfo);

    if (IS_FAIL(op_result)) {
        fprintf(stderr, "Error, cannot get device info.\n");
	RPlidarDriver::DisposeDriver(drv);
	exit(0);
    }

    // print out the device serial number, firmware and hardware version number..
    // cout << "Lidar Firmware Ver: " << (devinfo.firmware_version>>8) << "." << (devinfo.firmware_version & 0xFF)
    // 	 << endl
    // 	 << "Hardware Rev: " << (int)devinfo.hardware_version << endl;




    // start scan...
    drv->startMotor();
    drv->startScan();

    return drv;

}


vector<point> nodesToPoints(lidarNode nodes[], size_t count, bool zeroLowQuality){
  vector<point> result;
  for (int pos = 0; pos < (int)count ; ++pos) {
    double angle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f ;
    int quality = nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
    //cout << angle << " " << quality << endl;
    point p;
    p.theta= angle;
    if (quality > 0){
      double distance = nodes[pos].distance_q2/4.0f;
      p.r= distance;
      result.push_back(p);
    } else if (zeroLowQuality) {
      p.r = 0;
      result.push_back(p);
    }
  }
  return result;
}

vector<point> readLidar(RPlidarDriver * drv, bool zeroLowQuality){

    u_result op_result;
    lidarNode nodes[360*2];
    while(1){
        size_t   count = _countof(nodes);
        op_result = drv->grabScanData(nodes, count);
       	 if (IS_OK(op_result)) {
      	    drv->ascendScanData(nodes, count);
      	    return nodesToPoints(nodes, count, zeroLowQuality);
      	 }
      	 else{
      	    cout<<"bad lidar"<<endl;
      	 }
    }
}

vector<point> always360(const vector<point>& inPoints) {
  assert(inPoints.size() > 0 && inPoints.size() <= 360);
  // vector<point> outPoints;
  // double angle = inPoints[0].theta;
  // const double delta = 1;
  // int inIndex = 0;
  // outPoints.push_back(inPoints[inIndex]);
  // for(int i=0; i < 360-1; i++) {
  //   angle += delta;
  //   // if the next angle is close, use it
  //   //std::cout << angle << "  " << inPoints[inIndex+1].theta << std::endl;
  //   if ((angle - inPoints[inIndex+1].theta) < 0.5) {
  //     inIndex++;
  //     angle = inPoints[inIndex].theta;
  //   }
  //   outPoints.push_back(inPoints[inIndex]);
  // }
  // // std::cout << "in: " << inPoints.size() << "  out: " << outPoints.size() << std::endl;
  // // for(point p: inPoints) std::cout << p.r << " ";
  // // cout << std::endl;
  // // for(point p: outPoints) std::cout << p.r << " ";
  // // cout << std::endl;

  vector<point> outPoints = inPoints;
  point p;
  p.r = 0;
  p.theta=0;
  if (inPoints.size() < 360)
    outPoints.insert(outPoints.end(), 360-inPoints.size(), p);
  
  return outPoints;
}
