//
//  point.cpp
//  Lines
//
//  Created by Ben Shaffer on 3/14/15.
//  Copyright (c) 2015 Ben Shaffer. All rights reserved.
//

#include <fstream>
#include <iostream>
#include <cmath>
#include "point.hpp"

// double getX(point p){
//     return p.r*cos(p.theta*M_PI/180);
// }

// double getY(point p){
//     return p.r*sin(p.theta*M_PI/180);
// }

// point makePoint(double x, double y){
//     point p;
//     p.r=std::sqrt(x*x+y*y);
//     p.theta=std::atan2(y,x)*180/M_PI;
//     return p;
// }


// std::vector<point> readFromFile(std::string fileName){
//     std::vector<point> information;
//     std::ifstream fin(fileName.c_str());
//     double r;
//     double theta;
//     int index;
//     double dummy;
//     while(fin>>index>>dummy>>theta>>r>>dummy>>dummy>>dummy){
//         point p;
//         p.r=r;
//         p.theta=theta;
//         information.push_back(p);
//     }
//     return information;
// }

// void writeXY(std::string fileName, std::vector<point> points){
//     std::ofstream fout(fileName.c_str());
    
//     for(std::size_t i=0; i<points.size(); i++){
//         fout<< getX(points[i]) << " " << getY(points[i]) << std::endl;
//     }
//     fout.close();
// }

// double getSlope(point p1, point p2){
//     return ((getY(p1)-getY(p2))/(getX(p1)-getX(p2)));
// }

// std::vector<point> filter(std::vector<point> points){
// 	std::vector<point> result;
// 	for(size_t i=2; i<points.size()-1; i++){
		
// 		double slope1= getSlope((points[i-1]), (points[i-2])) ;
// 		double slope2= getSlope((points[i]), (points[i-1])) ;
// 		double slope3 = getSlope((points[i+1]), (points[i])) ;

// 		double slopeChange1=slope2-slope1; // 7
// 		double slopeChange2=slope3-slope1; // -2.5
// 		double percentDifference=(slopeChange1-slopeChange2)/slopeChange2;
// 		if(std::abs(percentDifference)<2.0){
// 			result.push_back(points[i]);
// 		} else {
// 		    std::cerr << "dropping " << getX(points[i]) << " " << getY(points[i]) << std::endl;
// 		}
// 	}
// 	return result;

// }

// double distanceBetweenPoints(point p1, point p2){
//     return std::sqrt((getX(p2)-getX(p1))*(getX(p2)-getX(p1))+((getY(p2)-getY(p1))*(getY(p2)-getY(p1))));
// }

// double dotProduct(point p1, point p2){
//     return (getX(p1)*getX(p2)+getY(p1)*getY(p2));
// }


// double lengthSquared(point p) {
//     return (getX(p)*getX(p) + getY(p) * getY(p));
// }

// double length(point p) {
//     return std::sqrt(lengthSquared(p));
// }

// point subtractPoints(point p1, point p2){
//     point finishPoint;
//     finishPoint= makePoint(getX(p1)-getX(p2), getY(p1)-getY(p2));
//     return finishPoint;
// }
