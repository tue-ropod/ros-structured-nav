#ifndef NAP_FUN_H
#define NAP_FUN_H

#include "napoleon_config.h"

class Point {
public:
	double x, y;
	Point(double xval, double yval);
	Point sub(Point b);
};

Point rotate_point(double cx, double cy, double angle, Point p);
Point coordGlobalToRopod(Point gp, Point rc, double ra);
double getSteering(Point local_wallpoint_front, Point local_wallpoint_rear);

#endif