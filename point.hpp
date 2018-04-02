//
//  point.h
//  Lines
//
//  Created by Ben Shaffer on 3/14/15.
//  Copyright (c) 2015 Ben Shaffer. All rights reserved.
//

#pragma once

#include <string>
#include <vector>

struct point{
    double r;
    double theta;
    bool operator ==(point p1){
        return (p1.r==r && p1.theta==theta);
    }

};

double getX(point p);
double getY(point p);
point makePoint(double x, double y);
std::vector<point> readFromFile(std::string fileName);
void writeXY(std::string fileName, std::vector<point> points);
double getSlope(point p1, point p2);
std::vector<point> filter(std::vector<point> points);
double distanceBetweenPoints(point p1, point p2);
double dotProduct(point p1, point p2);
double lengthSquared(point p);
double length(point p);
point subtractPoints(point p1, point p2);
