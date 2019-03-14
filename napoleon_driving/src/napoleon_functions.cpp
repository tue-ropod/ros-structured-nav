#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include "napoleon_config.h"
#include "napoleon_functions.h"
#include <ros/ros.h>
#include <algorithm>    // std::rotate
using namespace std;
#include <time.h>
#define MAX_DATE 12

// For all points except map points
Point::Point(double xval = 0.0, double yval = 0.0) {
	x = xval;
	y = yval;
}

// For map points
PointID::PointID(double xval = 0.0, double yval = 0.0, string idval = " ") {
	x = xval;
	y = yval;
	id = idval;
}

// For dynamic areas like entries
AreaQuad::AreaQuad(Point p0val, Point p1val, Point p2val, Point p3val) {
	p0 = p0val;
	p1 = p1val;
	p2 = p2val;
	p3 = p3val;
}

// For static areas like hallways and intersections
AreaQuadID::AreaQuadID(PointID p0val, PointID p1val, PointID p2val, PointID p3val, int idval, string typeval) {
	p0 = p0val;
	p1 = p1val;
	p2 = p2val;
	p3 = p3val;
	id = idval;
	type = typeval;
}

// Code can be cleaner if PointID inherits Point
Point Point::sub(Point b) {
	return Point(x - b.x, y - b.y);
}

Point Point::add(Point b) {
	return Point(x + b.x, y + b.y);
}

Point Point::sub(PointID b) {
	return Point(x - b.x, y - b.y);
}

Point Point::add(PointID b) {
	return Point(x + b.x, y + b.y);
}

PointID PointID::sub(Point b) {
	return PointID(x - b.x, y - b.y, id);
}

PointID PointID::add(Point b) {
	return PointID(x + b.x, y + b.y, id);
}

PointID PointID::sub(PointID b) {
	return PointID(x - b.x, y - b.y, id);
}

PointID PointID::add(PointID b) {
	return PointID(x + b.x, y + b.y, id);
}

bool AreaQuad::contains(Point q) {
	bool c = false;
	int i, j = 0;
	int nvert = 4;
	double vertx[4] = {p0.x, p1.x, p2.x, p3.x};
	double verty[4] = {p0.y, p1.y, p2.y, p3.y};
	for (i = 0, j = nvert-1; i < nvert; j = i++) {
		if ( ((verty[i]>q.y) != (verty[j]>q.y)) && (q.x < (vertx[j]-vertx[i]) * (q.y-verty[i]) / (verty[j]-verty[i]) + vertx[i]) ){
			c = !c;
		}
	}
	return c;
}

bool AreaQuadID::contains(Point q) {
	bool c = false;
	int i, j = 0;
	int nvert = 4;
	double vertx[4] = {p0.x, p1.x, p2.x, p3.x};
	double verty[4] = {p0.y, p1.y, p2.y, p3.y};
	for (i = 0, j = nvert-1; i < nvert; j = i++) {
		if ( ((verty[i]>q.y) != (verty[j]>q.y)) && (q.x < (vertx[j]-vertx[i]) * (q.y-verty[i]) / (verty[j]-verty[i]) + vertx[i]) ){
			c = !c;
		}
	}
	return c;
}

Point AreaQuadID::center() {
	Point a((p0.x+p1.x+p2.x+p3.x)/4, (p0.y+p1.y+p2.y+p3.y)/4);
	return a;
}

vector<string> AreaQuadID::getPointIDs() {
	vector<string> s {p0.id, p1.id, p2.id, p3.id};
	return s;
}

void printstringvec(vector<string> vec) 
{
	for (std::vector<string>::const_iterator i = vec.begin(); i != vec.end(); ++i) {
    	std::cout << *i << " ";
	}
	std::cout << std::endl;
}

Point rotate_point(Point c, double angle, Point p)
{
	double sinans = sin(angle);
	double cosans = cos(angle);

	// translate point back to origin:
	p.x -= c.x;
	p.y -= c.y;

	// rotate point
	double xnew = p.x * cosans - p.y * sinans;
	double ynew = p.x * sinans + p.y * cosans;

	// translate point back:
	p.x = xnew + c.x;
	p.y = ynew + c.y;
	return p;
}

Point coordGlobalToRopod(PointID gp, Point rc, double ra)
{
	// Function that transforms a point from the global coordinate system to the
	// coordinate system of the ropod[X up(pos), Y left(pos)].
	// gp global point[x, y]
	// rc ropod center[x, y]
	// ra ropod angle in global coordinate system[rad]
	Point vec_glob(gp.x-rc.x, gp.y-rc.y); // vector from ropod to point in global coord
	Point origin(0,0);
	Point P = rotate_point(origin, -ra, vec_glob); // [X, Y] position in ropod coordinates
	//cout << "In func coordGlobaltoRopod: " << P.x << ", " << P.y << endl;
	// X up, Y left, theta CCW, theta = 0 at X - axis
	return P;
}

Point coordGlobalToRopod(Point gp, Point rc, double ra)
{
	Point vec_glob(gp.x-rc.x, gp.y-rc.y); // vector from ropod to point in global coord
	Point origin(0,0);
	Point P = rotate_point(origin, -ra, vec_glob); // [X, Y] position in ropod coordinates
	return P;
}

double getSteering(Point local_wallpoint_front, Point local_wallpoint_rear, double tubewidth)
{
	// Function to determine steering action while cruising
	// ROPOD_LENGTH the length of the ropod[m]
	// SIZE_SIDE is the distance from the center to the side of the vehicle[m]
	// FEELER_SIZE is the length of the feeler, extending in front of the ropod
	// and triggering behavior changes - positioned above tr(top right) and tl[m]
	// local_wallpoint_front: point on right wall is a point that is to the
	// right of the ropod, can be any point(x, y) format[m]
	// local_wallpoint_rear : has to lie behind local_wallpoint_front, behind
	// as in relative to the direction in which the ropod wants to go
	// ropod_phi : current steering action[rad]
	// tubewidth is the tube width [m]
	// ENV_TCTW_SIZE is the distance when the ropod is too close to the wall[m]
	// ENV_TRNS_SIZE is the size of the transition area(starting from tctw)[m]
	// CARROT_LENGTH is how aggressive the ropod steers to the middle, it is
	// the scale for the vector that points forward from the middle of the road,
	// the bigger this value, the less aggressive the steering action[m]
	// fimdf: feelers in motion direction fraction, this determines whether
	// the feelers are pointed in the same direction as the wheels[1] or
	// just in front of the ropod[0] or something in between[0, 1]
	if (tubewidth < 2 * (SIZE_SIDE + ENV_TCTW_SIZE + ENV_TRNS_SIZE))
	{
		cout << "Corridor too small, or transition and correction zone too big" << endl;
	}

	// Feelers in front of the ropod
	Point local_fl(ROPOD_LENGTH / 2 + FEELER_SIZE, SIZE_SIDE);
	Point local_fr(ROPOD_LENGTH / 2 + FEELER_SIZE, -SIZE_SIDE);
	Point origin(0,0);

	double local_wall_angle = atan2(local_wallpoint_front.y - local_wallpoint_rear.y, local_wallpoint_front.x - local_wallpoint_rear.x);

	Point fl_env_0, rm_env_0, fr_env_0, rb_env_0;

	fl_env_0 = rotate_point(origin, -local_wall_angle, local_fl);
	//fl_env_0 = rotate_point(local_fl, zerovec, -local_wall_angle);               // Feeler left @ env at theta = 0
	//rm_env_0 = rotate_point(local_wallpoint_front, zerovec, -local_wall_angle);	// Middle of lane @ env at theta = 0
	rm_env_0 = rotate_point(origin, -local_wall_angle, local_wallpoint_front);
	rm_env_0.y += tubewidth;
	double dist_left = rm_env_0.y - fl_env_0.y;

	//fr_env_0 = rotate_point(local_fr, zerovec, -local_wall_angle);				// Feeler right @ env at theta = 0
	fr_env_0 = rotate_point(origin, -local_wall_angle, local_fr);
	//rb_env_0 = rotate_point(local_wallpoint_rear, zerovec, -local_wall_angle);	// Wall at right side @ env at theta = 0
	rb_env_0 = rotate_point(origin, -local_wall_angle, local_wallpoint_rear);
	double dist_right = fr_env_0.y - rb_env_0.y;								// Y distance from feeler right to the wall(neg if beyond wall)

	double dist_ropod_center_to_rightwall = -rb_env_0.y;						// Y distance from ropod center to right wall

	double to_middle_size = tubewidth/2 - dist_ropod_center_to_rightwall;					// Y Distance from ropod center to middle of road @ env at theta = 0
	double to_middle_vec[2] = { to_middle_size * -sin(local_wall_angle), to_middle_size * cos(local_wall_angle) };	// Vector from ropod center to middle of road
	double to_front_vec[2] = { CARROT_LENGTH * cos(local_wall_angle), CARROT_LENGTH*sin(local_wall_angle) };	// Vector from middle of road to a point further on the road
	double steer_vec[2] = { to_front_vec[0] + to_middle_vec[0], to_front_vec[1] + to_middle_vec[1] };               // Together, these vectors form the steering vector

	// Phi_0 can be used when we are comfortably within the middle of the road
	// Phi_tctw will be used when the feelers are really close to the wall
	// In the transition zone we will do a combination of Phi_0 and Phi_tctw
	double phi_0 = 0.0;
	double phi_tctw = atan2(steer_vec[1], steer_vec[0]); // Determine the angle of this steering vector
	double phi = phi_0;

	if (dist_right < ENV_TCTW_SIZE && dist_left < ENV_TCTW_SIZE)
	{
		//warning('Poor condition: ropod is too close to left AND right wall');
		phi = phi_tctw;
	}
	else if (dist_right < ENV_TCTW_SIZE)
	{
		// disp('Ropod is too close to right side, steer back to middle');
		phi = phi_tctw;
	}
	else if (dist_left < ENV_TCTW_SIZE)
	{
		// disp('Ropod is too close to left side, steer back to middle');
		phi = phi_tctw;
	}
	else if (dist_right < ENV_TCTW_SIZE + ENV_TRNS_SIZE && dist_left < ENV_TCTW_SIZE + ENV_TRNS_SIZE)
	{
		//warning('Poor condition: ropod is in transition zone on both sides');
		phi = phi_tctw;
	}
	else if (dist_right < ENV_TCTW_SIZE + ENV_TRNS_SIZE)
	{
		// disp('Ropod is in transition zone on the right side');
		double frac_in_trans = 1 - (dist_right - ENV_TCTW_SIZE) / ENV_TRNS_SIZE;
		phi = frac_in_trans * phi_tctw + (1 - frac_in_trans)*phi_0;
	}
	else if (dist_left < ENV_TCTW_SIZE + ENV_TRNS_SIZE)
	{
		// disp('Ropod is in transition zone on the left side');
		double frac_in_trans = 1 - (dist_left - ENV_TCTW_SIZE) / ENV_TRNS_SIZE;
		phi = frac_in_trans * phi_tctw + (1 - frac_in_trans)*phi_0;
	}
	else
	{
		// disp('Ropod is comfortably in the middle of the lane, no steering action necessary');
		phi = phi_0;
	}

	return phi;
}

double getSteeringTurn(Point local_pivot, bool dir_cw, Point local_wallpoint_front, Point local_wallpoint_rear) {
    // Function to determine steering action while rotating around a point
    // aka taking a turn
    // dir_cw: direction 0 = CCW, 1 = CW rotation

    // Feelers in front of the ropod
    Point local_lt(ROPOD_LENGTH/2, SIZE_SIDE);
	Point local_rt(ROPOD_LENGTH/2,-SIZE_SIDE);
	Point origin(0,0);
	double dist = 0, phi = 0, to_middle_size = 0;
    
    double phi_0 = steerAroundPoint(local_pivot, dir_cw);
	Point local_fl(FEELER_SIZE*cos(phi_0),FEELER_SIZE*sin(phi_0));
	Point local_fr(FEELER_SIZE*cos(phi_0),FEELER_SIZE*sin(phi_0));
	local_fl = local_fl.add(local_lt);
	local_fr = local_fr.add(local_rt);
    
    double local_wall_angle = atan2(local_wallpoint_front.y-local_wallpoint_rear.y,local_wallpoint_front.x-local_wallpoint_rear.x);
	Point fl_env_0 = rotate_point(origin, -local_wall_angle, local_fl); 	// Feeler left @ env at theta = 0
    Point rb_env_0 = rotate_point(origin, -local_wall_angle, local_wallpoint_rear);   // Wall at left side @ env at theta = 0
    double dist_left = rb_env_0.y-fl_env_0.y;    // Y distance from feeler left to the wall (neg if beyond wall)

    Point fr_env_0 = rotate_point(origin, -local_wall_angle, local_fr);   	// Feeler right @ env at theta = 0
    //Point rb_env_0 = rotate_point(origin, -local_wall_angle, local_wallpoint_rear);   // Wall at right side @ env at theta = 0
    double dist_right = fr_env_0.y-rb_env_0.y;  // Y distance from feeler right to the wall (neg if beyond wall)
    double dist_ropod_center_to_wall = -rb_env_0.y;      // Y distance from ropod center to right wall

    if (dir_cw) { // CW
        // Follow left wall if too close to wall when turning right
        // to_middle_size = dist_ropod_center_to_wall-tubewidth/2;
		to_middle_size = dist_ropod_center_to_wall-FOLLOW_WALL_DIST_TURNING;
		//to_middle_size = FOLLOW_WALL_DIST_TURNING-dist_ropod_center_to_wall;
    } else { //CCW
        // Follow right wall if too close to wall when turning left
		//to_middle_size = dist_ropod_center_to_wall-FOLLOW_WALL_DIST_TURNING;
		to_middle_size = FOLLOW_WALL_DIST_TURNING-dist_ropod_center_to_wall;
        // to_middle_size = tubewidth/2-dist_ropod_center_to_wall;
	}
	Point to_middle_vec(to_middle_size*-sin(local_wall_angle),to_middle_size*cos(local_wall_angle));
    Point to_front_vec(CARROT_LENGTH*cos(local_wall_angle), CARROT_LENGTH*sin(local_wall_angle));  // Vector from middle of road to a point further on the road
    Point steer_vec(to_front_vec.x + to_middle_vec.x, to_front_vec.y + to_middle_vec.y);
	//steer_vec = steer_vec.add(to_middle_vec); // Together, these vectors form the steering vector

    if (dir_cw) { 	// CW, left side of ropod will be closest to wall
        dist = dist_left;
		
	} else {       	// CCW, right side of ropod will be closest to wall
        dist = dist_right;
	}
    
    // Phi_0 can be used when we are comfortably within the middle of the road
    // Phi_tctw will be used when the feelers are really close to the wall
    // In the transition zone we will do a combination of Phi_0 and Phi_tctw
    // phi_0 = 0;
    double phi_tctw = atan2(steer_vec.y,steer_vec.x); // Determine the angle of this steering vector

    if (dist < ENV_TCTW_SIZE) {
        //ROS_INFO("Ropod is too close to wall, steer back to middle");
        phi = phi_tctw;
		//ROS_INFO("to_middle_size: %f", to_middle_size);
		//ROS_INFO("dist left: %f", dist_left);
	} else if (dist < ENV_TCTW_SIZE+ENV_TRNS_SIZE) {
        //ROS_INFO("Ropod is in transition zone at the wall");
        double frac_in_trans = 1-(dist-ENV_TCTW_SIZE)/ENV_TRNS_SIZE;
        phi = frac_in_trans*phi_tctw+(1-frac_in_trans)*phi_0;
	} else {
        //ROS_INFO("Ropod is steering without collision predicted");
        phi = phi_0;
	}
	return phi;
}

double steerAroundPoint(Point local_pivot, bool dir_cw) {
    // local pivot: rotation point in ropod coordinates [x,y], 
    // dir_cw: direction 0 = CCW, 1 = CW
    Point A(0,0);               // Ropod center local
    Point B(-D_AX,0);   		// Rearaxle local
    Point C = local_pivot;      // Rotation point in local coords
	double phi;

    Point Arot = rotate_point(B, M_PI/2, A);
	Arot = Arot.sub(B);
    Point Crot = rotate_point(B, M_PI/2, C);
	Crot = Crot.sub(B);
	Point Diff = Arot;
	Diff = Arot.sub(Crot);

    if (dir_cw) {	// CW (turn right)
		phi = atan2(Diff.y,Diff.x)-M_PI;
	} else {		// CCW (turn left)
        phi = atan2(Diff.y,Diff.x);
	}
	return phi;
}

double distToSegment(Point p, PointID v, PointID w) {
    // P point, v, w points of line segmens
	Point v_noid(v.x,v.y);
	Point w_noid(w.x,w.y);
    return sqrt(distToSegmentSquared(p, v_noid, w_noid));
}

double distToSegment(Point p, Point v, Point w) {
    // P point, v, w points of line segmens
    return sqrt(distToSegmentSquared(p, v, w));
}

double dist2(Point v, Point w) {
    // Returns square distance between 2 points
    return (v.x - w.x)*(v.x - w.x) + (v.y - w.y)*(v.y - w.y);
}

double distToSegmentSquared(Point p, Point v, Point w) {
    double l2 = dist2(v, w);
    if (l2 == 0) {
        return dist2(p, v);
	} else {
        double t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;
        t = max(0.0, min(1.0, t));
        return dist2(p, Point(v.x + t * (w.x - v.x), v.y + t * (w.y - v.y)));
    }
}

bool do_lines_intersect(Point p0, Point p1, Point p2, Point p3) {
    // Returns 1 if the lines intersect, otherwise 0.
    // Function inspiration: https://stackoverflow.com/questions/563198/ ...
    // how-do-you-detect-where-two-line-segments-intersect#answer-1968345
    double s1_x = p1.x - p0.x;     
    double s1_y = p1.y - p0.y;
    double s2_x = p3.x - p2.x;     
    double s2_y = p3.y - p2.y;
    double s = (-s1_y * (p0.x - p2.x) + s1_x * (p0.y - p2.y)) / (-s2_x * s1_y + s1_x * s2_y);
    double t = ( s2_x * (p0.y - p2.y) - s2_y * (p0.x - p2.x)) / (-s2_x * s1_y + s1_x * s2_y);
    if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
        return true;
	} else {
        return false; // No intersection
	}
}

bool do_shapes_overlap(Point obj1p0, Point obj1p1, Point obj1p2, Point obj1p3, Point obj2p0, Point obj2p1, Point obj2p2, Point obj2p3) {
    // Returns 1 if shapes overlap, 0 otherwise
    // Shapes have to be defined as [x1, y1; x2, y2, etc]
    // where the points represent the corners of the shape.
    // This method assumed that the shapes are closed.
	// PS: this is very very ugly code. Sorry for your eyes & brain. Time pressure at the end of projects...
	// double shape1[4][2] {{obj1p0.x, obj1p0.y}, {obj1p1.x, obj1p1.y}, {obj1p2.x, obj1p2.y}, {obj1p3.x, obj1p3.y}};
	// double shape2[4][2] {{obj2p0.x, obj2p0.y}, {obj2p1.x, obj2p1.y}, {obj2p2.x, obj2p2.y}, {obj2p3.x, obj2p3.y}};
	vector<Point> sh1 {obj1p0, obj1p1, obj1p2, obj1p3};
	vector<Point> sh2 {obj2p0, obj2p1, obj2p2, obj2p3};
    bool colliding = false;
	bool inter = false;
	int pnxt, qnxt;
	int shape1size = 4; // Only working for rectangles for now
	int shape2size = 4; // Only working for rectangles for now
	Point p0, p1, p2, p3;
    // For all lines of shape 1
    for (int q = 0; q < shape1size; ++q) {
        qnxt = q+1;
		if (qnxt > shape1size-1) { qnxt = 0; }
        p0 = sh1[q]; p1 = sh1[qnxt];
        //p0.x = shape1[q][0]; p0.y = shape1[q][1];
		//p1.x = shape1[qnxt][0]; p1.y = shape1[qnxt][1];

        // Check if they intersect with all lines of shape 2
        for (int p = 0; p < shape2size; ++p) {
			pnxt = p+1;
            if (pnxt > shape2size-1) { pnxt = 0; }
			p2 = sh2[p]; p3 = sh2[pnxt];
            //p2.x = shape2[p][0]; p2.y = shape2[p][1];
            //p3.x = shape2[pnxt][0]; p3.y = shape2[pnxt][1];

            inter = do_lines_intersect(p0, p1, p2, p3);
            if (inter) {
                colliding = true;
			}
		}
    }
    
    // Check special case where one shape encloses the other
    // Check is performed with ray casting, background:
    // https://en.wikipedia.org/wiki/Point_in_polygon
    // https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon
    // if (colliding) {
    //     double e = 0.1; // Padding for ray casting [m]
    //     bool c1 = false, c2 = false; // Keep track of inside or outside
    //     double s1p1x = obj1p0.x;
    //     double s1p1y = obj1p0.y;
    //     double s2p1x = obj2p0.x;
    //     double s2p1y = obj2p0.y;
	// 	double s1xmin = 99.9;
	// 	double s2xmin = 99.9;
	// 	// C++ doesn't have an operator to get a column, so getting the minimum value this way...
	// 	for (int minc = 0; minc < shape1size; ++minc) {
	// 		double curval = shape1[minc][0];
    //     	if (curval < s1xmin) {
	// 			s1xmin = curval;
	// 		}
	// 	}
	// 	for (int minc = 0; minc < shape2size; ++minc) {
	// 		double curval = shape2[minc][0];
    //     	if (curval < s2xmin) {
	// 			s2xmin = curval;
	// 		}
	// 	}
    //     double ray_s1[4] = {s1xmin-e, s2p1y, s2p1x, s2p1y};
    //     // double r1_p2_x = ray_s1[0];
    //     // double r1_p2_y = ray_s1[1];
    //     // double r1_p3_x = ray_s1[2];
    //     // double r1_p3_y = ray_s1[3];
	// 	Point r1p2(ray_s1[0],ray_s1[1]);
	// 	Point r1p3(ray_s1[2],ray_s1[3]);
    //     double ray_s2[4] = {s2xmin-e, s1p1y, s1p1x, s1p1y};
    //     // double r2_p2_x = ray_s2[0];
    //     // double r2_p2_y = ray_s2[1];
    //     // double r2_p3_x = ray_s2[2];
    //     // double r2_p3_y = ray_s2[3];
	// 	Point r2p2(ray_s2[0],ray_s2[1]);
	// 	Point r2p3(ray_s2[2],ray_s2[3]);
    //     for (int q = 0; q < shape1size; ++q) {
	// 		int qnxt = q+1;
    //         if (q+1 > shape1size-1) { qnxt = 0; }
    //         // double p0_x = shape1[q,1];
    //         // double p0_y = shape1[q,2];
	// 		Point p0(shape1[q][0],shape1[q][1]);
	// 		Point p1(shape1[qnxt][0],shape1[qnxt][1]);
    //         // double p1_x = shape1[qnxt,1];
    //         // double p1_y = shape1[qnxt,2];
    //         bool inter = do_lines_intersect(p0, p1, r1p2, r1p3);
    //         if (inter) {
    //             c1 = !c1; // If c1 was 0, it becomes 1, if c1 was 1 it becomes 0
    //         }
	// 	}
    //     for (int p = 0; p < shape2size-1; ++p) {
    //         int pnxt = p+1;
    //         if (p+1 > shape2size) { int pnxt = 0; }
	// 		Point p0(shape2[p][0],shape2[p][1]);
	// 		Point p1(shape2[pnxt][0],shape2[pnxt][1]);
    //         // double p0_x = shape2[p,1];
    //         // double p0_y = shape2[p,2];
    //         // double p1_x = shape2[pnxt,1];
    //         // double p1_y = shape2[pnxt,2];
    //         bool inter = do_lines_intersect(p0, p1, r2p2, r2p3);
    //         if (inter) {
    //             c2 = !c2; // If c2 was 0, it becomes 1, if c2 was 1 it becomes 0
    //             //disp('Shape 1 lies completely in shape 2');
	// 		}
	// 	}
    //     if (c1 || c2) { // If c1 or c2 is true, then one shape lies within the other
    //         colliding = true;
    //         // disp('SHAPES ENCLOSED EACH OTHER WHOO');
	// 	}
	// }
	return colliding;
}

bool do_shapes_overlap(double shape1[][2], double shape2[][2]) {
    // Returns 1 if shapes overlap, 0 otherwise
    // Shapes have to be defined as [x1, y1; x2, y2, etc]
    // where the points represent the corners of the shape.
    // This method assumed that the shapes are closed.
    bool colliding = false;
	int shape1size = 4; // Only working for rectangles for now
	int shape2size = 4; // Only working for rectangles for now
    // For all lines of shape 1
    for (int q = 0; q < shape1size; ++q) {
        int qnxt = q+1;
		if (q+1 > shape1size-1) { qnxt = 0; }
        Point p0(shape1[q][0],shape1[q][1]);
        Point p1(shape1[qnxt][0],shape1[qnxt][1]);

        // Check if they intersect with all lines of shape 2
        for (int p = 0; p < shape2size; ++p) {
			int pnxt = p+1;
            if (p+1 > shape2size-1) { int pnxt = 0; }
            Point p2(shape2[p][0],shape2[p][1]);
            Point p3(shape2[pnxt][0],shape2[pnxt][1]);

            bool inter = do_lines_intersect(p0, p1, p2, p3);
            if (inter) {
                colliding = true;
			}
		}
    }
    
    // Check special case where one shape encloses the other
    // Check is performed with ray casting, background:
    // https://en.wikipedia.org/wiki/Point_in_polygon
    // https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon
    if (colliding) {
        double e = 0.1; // Padding for ray casting [m]
        bool c1 = false, c2 = false; // Keep track of inside or outside
        double s1p1x = shape1[0][0];
        double s1p1y = shape1[0][1];
        double s2p1x = shape2[0][0];
        double s2p1y = shape2[0][1];
		double s1xmin = 99.9;
		double s2xmin = 99.9;
		// C++ doesn't have an operator to get a column, so getting the minimum value this way...
		for (int minc = 0; minc < shape1size; ++minc) {
			double curval = shape1[minc][0];
        	if (curval < s1xmin) {
				s1xmin = curval;
			}
		}
		for (int minc = 0; minc < shape2size; ++minc) {
			double curval = shape2[minc][0];
        	if (curval < s2xmin) {
				s2xmin = curval;
			}
		}
        double ray_s1[4] = {s1xmin-e, s2p1y, s2p1x, s2p1y};
        // double r1_p2_x = ray_s1[0];
        // double r1_p2_y = ray_s1[1];
        // double r1_p3_x = ray_s1[2];
        // double r1_p3_y = ray_s1[3];
		Point r1p2(ray_s1[0],ray_s1[1]);
		Point r1p3(ray_s1[2],ray_s1[3]);
        double ray_s2[4] = {s2xmin-e, s1p1y, s1p1x, s1p1y};
        // double r2_p2_x = ray_s2[0];
        // double r2_p2_y = ray_s2[1];
        // double r2_p3_x = ray_s2[2];
        // double r2_p3_y = ray_s2[3];
		Point r2p2(ray_s2[0],ray_s2[1]);
		Point r2p3(ray_s2[2],ray_s2[3]);
        for (int q = 0; q < shape1size; ++q) {
			int qnxt = q+1;
            if (q+1 > shape1size-1) { qnxt = 0; }
            // double p0_x = shape1[q,1];
            // double p0_y = shape1[q,2];
			Point p0(shape1[q][0],shape1[q][1]);
			Point p1(shape1[qnxt][0],shape1[qnxt][1]);
            // double p1_x = shape1[qnxt,1];
            // double p1_y = shape1[qnxt,2];
            bool inter = do_lines_intersect(p0, p1, r1p2, r1p3);
            if (inter) {
                c1 = !c1; // If c1 was 0, it becomes 1, if c1 was 1 it becomes 0
            }
		}
        for (int p = 0; p < shape2size-1; ++p) {
            int pnxt = p+1;
            if (p+1 > shape2size) { int pnxt = 0; }
			Point p0(shape2[p][0],shape2[p][1]);
			Point p1(shape2[pnxt][0],shape2[pnxt][1]);
            // double p0_x = shape2[p,1];
            // double p0_y = shape2[p,2];
            // double p1_x = shape2[pnxt,1];
            // double p1_y = shape2[pnxt,2];
            bool inter = do_lines_intersect(p0, p1, r2p2, r2p3);
            if (inter) {
                c2 = !c2; // If c2 was 0, it becomes 1, if c2 was 1 it becomes 0
                //disp('Shape 1 lies completely in shape 2');
			}
		}
        if (c1 || c2) { // If c1 or c2 is true, then one shape lies within the other
            colliding = true;
            // disp('SHAPES ENCLOSED EACH OTHER WHOO');
		}
	}
	return colliding;
}

bool does_line_intersect_shape(Point p0, Point p1, double shape[][2]) {
    bool intersecting = false;

	int shapesize = 4; // Only working for rectangles for now
    // Check if the line intersects with all lines of the shape
    for (int p = 0; p < shapesize; ++p) {
        int pnxt = p+1;
		if (p+1 > shapesize-1) { int pnxt = 0; }
        Point p2(shape[p][0],shape[p][1]);
        Point p3(shape[pnxt][0],shape[pnxt][1]);

        if (do_lines_intersect(p0, p1, p2, p3)) {
            intersecting = true;
		}
	}
	return intersecting;
}

// Case where circle is completely within shape is not considered as this is
// not very likely to occur with the sizes of the objects, plus the velocity
// adaptation will also work of a few prediction samples say 'no collision',
// as long as the other samples notice a collision.
bool doesShapeCollideWithCircle(double shape[][2], Point c, double r) {
	int shapesize = 4; 	// Only working for rectangles for now
    double smallest_dist = 999.9;
	double dts;
    for (int q = 0; q < shapesize; ++q) {
		int qnxt = q+1;
		if (q+1 > shapesize-1) { qnxt = 0; }
		Point p0(shape[q][0],shape[q][1]);
		Point p1(shape[qnxt][0],shape[qnxt][1]);
        dts = distToSegment(c, p0, p1);
        if (dts < smallest_dist) {
			smallest_dist = dts;
		}
	}

    if (smallest_dist > r) {
        return false; 	// Too far away to collide
	} else {
        return true; 	// Colliding
	}
}

AreaQuad generateEntry(int hallwayID, int interID, double e_length, vector<AreaQuadID> arealist, vector<PointID> pointlist) {
    AreaQuadID HALLWAY_OBJ = getAreaByID(hallwayID,arealist);
    AreaQuadID INTER_OBJ = getAreaByID(interID,arealist);
    vector<string> wall {getWallPointsTowardsB(HALLWAY_OBJ,INTER_OBJ)};
	vector<string> leftwall {getWallPointsAwayFromB(HALLWAY_OBJ,INTER_OBJ)};
    // Hallway end are the two points connected to the intersection
	vector<string> HALLWAY_end {getCommonPoints(HALLWAY_OBJ,INTER_OBJ)};
    vector<string> HALLWAY_points = HALLWAY_OBJ.getPointIDs();
	int qmax = HALLWAY_points.size();
	vector<int> indices;
	// To find the hallway start we want the two other points,
	// so we are looking for the two points that are not in the common set,
	// this is then the hallway start.
	for (int q = 0; q < qmax; ++q) {
		string currentpoint = HALLWAY_points[q];
		if (currentpoint.compare(HALLWAY_end[0]) != 0 || currentpoint.compare(HALLWAY_end[1]) == 0) {
			indices.push_back (q);
		}
	}
    vector<string> HALLWAY_start {HALLWAY_points[indices[0]], HALLWAY_points[indices[1]]};
    // PointID s1 = getPointByID(HALLWAY_start[0], pointlist);
    // PointID s2 = getPointByID(HALLWAY_start[1], pointlist);
    // PointID e1 = getPointByID(HALLWAY_end[0], pointlist);
    // PointID e2 = getPointByID(HALLWAY_end[1], pointlist);
    PointID s_wall = getPointByID(wall[0], pointlist);
    PointID e_wall = getPointByID(wall[1], pointlist);
	PointID s_leftwall = getPointByID(leftwall[1], pointlist);
	PointID e_leftwall = getPointByID(leftwall[0], pointlist);
	Point s_wall_idless(s_wall.x, s_wall.y);
	Point e_wall_idless(e_wall.x, e_wall.y);
	Point s_leftwall_idless(s_leftwall.x, s_leftwall.y);
	Point e_leftwall_idless(e_leftwall.x, e_leftwall.y);
    // Point s_mid((s1.x+s2.x)/2, (s1.y+s2.y)/2);
    // Point e_mid((e1.x+e2.x)/2, (e1.y+e2.y)/2);
    double wall_angle = atan2(e_wall.y-s_wall.y,e_wall.x-s_wall.x);
    double leftwall_angle = atan2(e_leftwall.y-s_leftwall.y,e_leftwall.x-s_leftwall.x);
    Point start_entry_leftwall(e_leftwall.x-e_length*cos(leftwall_angle), e_leftwall.y-e_length*sin(leftwall_angle));
    Point start_entry_wall(e_wall.x-e_length*cos(wall_angle), e_wall.y-e_length*sin(wall_angle));
    AreaQuad entry(start_entry_wall, e_wall_idless, e_leftwall_idless, start_entry_leftwall);
	return entry;
}

AreaQuadID getAreaByID(int wantedID, vector<AreaQuadID> arealist) {
	int listlength = arealist.size();
	for (int i = 0; i < listlength; ++i) {
        if (arealist[i].id == wantedID) {
            return arealist[i];
		}
	}
}

vector<string> getWallPointsTowardsB(AreaQuadID A, AreaQuadID B) {
	vector<string> common_points_AB = getCommonPoints(A,B);	// (e.g. points S, T)
    vector<string> OBJA_point_IDs = A.getPointIDs();	// (e.g. points R, S, T, AG)
	//vector<string> points;
	// printstringvec(common_points_AB);
	// printstringvec(OBJA_point_IDs);
    if (common_points_AB.size() < 2) {   // Need 2 common points to be adjacent
        ROS_INFO("Assignment not executable since some areas are not adjacent");
	} else {
        int ind = 0; // = find(ismember(OBJA_point_IDs,common_points_AB{1}));
		int qmax = OBJA_point_IDs.size();
		for (int q = 0; q < qmax; ++q) {
			if (OBJA_point_IDs[q].compare(common_points_AB[0]) == 0) {
				ind = q; 
			}
		} 
		// The array is shifted so that it begins at common_points_AB[0] (e.g. S (could also be T))
		rotate(OBJA_point_IDs.begin(), OBJA_point_IDs.begin() + ind, OBJA_point_IDs.end());

		if (OBJA_point_IDs[1].compare(common_points_AB[0]) == 0 || OBJA_point_IDs[1].compare(common_points_AB[1]) == 0) {
			// If the NEXT point, common_points_AB[1], is a common point with area B
			// We shift the points one position to the left
			rotate(OBJA_point_IDs.begin(), OBJA_point_IDs.begin() + 1, OBJA_point_IDs.end());
		}	// If the next point is not a common point with B, we already have
			// the desired point as first point in the array
		// We now know that OBJA_point_IDs[0] -> OBJA_point_IDs[1] are the wall points
		// away from area B, and that OBJA_point_IDs[2] -> OBJA_point_IDs[3] are the
		// wall points towards area B.
    }
	vector<string> s = {OBJA_point_IDs[2], OBJA_point_IDs[3]};
	return s;
}

vector<string> getWallPointsAwayFromB(AreaQuadID A, AreaQuadID B) {
	vector<string> common_points_AB = getCommonPoints(A,B);	// (e.g. points S,T)
    vector<string> OBJA_point_IDs = A.getPointIDs();	// (e.g. points R, S, T, AG)
	vector<string> points;
	vector<string> s;
    if (common_points_AB.size() < 2) {   // Need 2 common points to be adjacent
        ROS_INFO("Assignment not executable since some areas are not adjacent");
	} else {
        int ind; // = find(ismember(OBJA_point_IDs,common_points_AB{1}));
		int qmax = OBJA_point_IDs.size();
		for (int q = 0; q < qmax; ++q) {
			if (OBJA_point_IDs[q].compare(common_points_AB[0]) == 0) {
				ind = q;
			}
		}
		// The array is shifted so that it begins at common_points_AB[0] (e.g. S (could also be T))
		rotate(OBJA_point_IDs.begin(), OBJA_point_IDs.begin() + ind, OBJA_point_IDs.end());
		//rotate(OBJA_point_IDs.rbegin(), OBJA_point_IDs.rbegin() -ind, OBJA_point_IDs.rend());

		if (OBJA_point_IDs[1].compare(common_points_AB[0]) == 0 || OBJA_point_IDs[1].compare(common_points_AB[1]) == 0) {
			// If the NEXT point, common_points_AB[1], is a common point with area B
			// We shift the points one position to the left
			rotate(OBJA_point_IDs.begin(), OBJA_point_IDs.begin() + 1, OBJA_point_IDs.end());
		}	// If the next point is not a common point with B, we already have
			// the desired point as first point in the array
		// We now know that OBJA_point_IDs[0] -> OBJA_point_IDs[1] are the wall points
		// away from area B, and that OBJA_point_IDs[2] -> OBJA_point_IDs[3] are the
		// wall points towards area B.
		s = {OBJA_point_IDs[0], OBJA_point_IDs[1]};
    }
	return s;
}

vector<string> getCommonPoints(AreaQuadID A, AreaQuadID B) {
    vector<string> common;	
	vector<string> ids1 {A.p0.id, A.p1.id, A.p2.id, A.p3.id};
	vector<string> ids2 {B.p0.id, B.p1.id, B.p2.id, B.p3.id};
	int ids1len = ids1.size();
	int ids2len = ids2.size();
    for (int i = 0; i < ids1len; ++i) {
        for (int j = 0; j < ids2len; ++j) {
            if (ids1[i].compare(ids2[j]) == 0) {
                common.push_back(ids1[i]);
			}
		}
	}
    return common;
}

PointID getPointByID(string wantedID, vector<PointID> pointlist) {
	int listlength = pointlist.size();
	for (int i = 0; i < listlength; ++i) {
		if (wantedID.compare(pointlist[i].id) == 0) {
            return pointlist[i];
		}
    }
}

vector<string> getPointsForTurning(AreaQuadID OBJ1, AreaQuadID OBJ2, AreaQuadID OBJ3, vector<string> OBJ1TASK) {
    vector<string> OBJ2_point_IDs = OBJ2.getPointIDs();
	int ind; // ind = find(ismember(OBJ2_point_IDs,OBJ1TASK{2}));
	int qmax = OBJ2_point_IDs.size();
	for (int q = 0; q < qmax; ++q) {
		if (OBJ2_point_IDs[q].compare(OBJ1TASK[1]) == 0) {
			ind = q;
		}
	}
    // shiftedOBJ2 = circshift(OBJ2_point_IDs,-ind);
	// The array is shifted so that it begins at OBJ1TASK[1], which is the bottom right point
	//rotate(OBJ2_point_IDs.rbegin(), OBJ2_point_IDs.rbegin() -ind, OBJ2_point_IDs.rend());
	rotate(OBJ2_point_IDs.begin(), OBJ2_point_IDs.begin() + ind, OBJ2_point_IDs.end());

    vector<string> common_points_12 = getCommonPoints(OBJ1,OBJ2);
    vector<string> common_points_23 = getCommonPoints(OBJ2,OBJ3);
    string pivotpoint;
	bool foundpivot = false;
	vector<string> intersectiontask;
	vector<string> next_wall_hallway;
    
	for (int i = 0; i < common_points_12.size(); ++i) {
		for (int j = 0; j < common_points_23.size(); ++j) {
			if (common_points_12[i].compare(common_points_23[j]) == 0) {
				pivotpoint = common_points_12[i];
				foundpivot = true;
			}
		}
	}
    
    //determinedirpoints
    if (foundpivot) {
        // disp('We are dealing with a corner');
		int index_dir; // index_dir = find(ismember(shiftedOBJ2,pivotpoint));
		int pmax = OBJ2_point_IDs.size();
		for (int p = 0; p < pmax; ++p) {
			if (OBJ2_point_IDs[p].compare(pivotpoint) == 0) {
				index_dir = p;
			}
		}

        if (index_dir == 0) {
            next_wall_hallway = getWallPointsTowardsB(OBJ3,OBJ2);
            // disp('Corner/intersection: turn right');
            // disp(['Use node: ', shiftedOBJ2{3}, ' as rear wall point']);
            // disp(['Use node: ', shiftedOBJ2{2}, ' as front wall point']);
            // disp(['Use node: ', shiftedOBJ2{1}, ' as pivot']);
            // disp('-------------------------------');
            // Wallpoint rear, wallpoint front, pivoting point
            intersectiontask = {OBJ2_point_IDs[2], OBJ2_point_IDs[1], next_wall_hallway[1], next_wall_hallway[0], OBJ2_point_IDs[0],"right"};
		} else if (index_dir == 3) {
            next_wall_hallway = getWallPointsAwayFromB(OBJ3,OBJ2);
            // disp('Corner/intersection: turn left');
            // disp(['Use node: ', shiftedOBJ2{2}, ' as rear wall point']);
            // disp(['Use node: ', shiftedOBJ2{3}, ' as front wall point']);
            // disp(['Use node: ', shiftedOBJ2{4}, ' as pivot']);
            // disp('-------------------------------');
            intersectiontask = {OBJ2_point_IDs[1], OBJ2_point_IDs[2], next_wall_hallway[0], next_wall_hallway[1], OBJ2_point_IDs[3],"left"};
            // Wallpoint rear, wallpoint front, pivoting point
		} else {
            ROS_INFO("Cannot turn around 2nd or 3rd node");
		}
	} else {
        // disp('Intersection: go straight');
        // disp(['Follow (fictious) wall: ', shiftedOBJ2{1}, ' -> ', shiftedOBJ2{2}]);
        // disp(['Use node: ', shiftedOBJ2{1}, ' as rear wall point']);
        // disp(['Use node: ', shiftedOBJ2{2}, ' as front wall point']);
        // disp('-------------------------------');
        intersectiontask = {OBJ2_point_IDs[0], OBJ2_point_IDs[1]};
        // Wallpoint rear, wallpoint front
	}
	return intersectiontask;
}

vector<string> getWalls(int id_OBJ1, int id_OBJ2, int id_OBJ3, vector<AreaQuadID> arealist) {
    AreaQuadID A = getAreaByID(id_OBJ1, arealist);
    AreaQuadID B = getAreaByID(id_OBJ2, arealist);
    AreaQuadID C = getAreaByID(id_OBJ3, arealist);
	int ind, qmax;
    
    vector<string> common_points_AB = getCommonPoints(A,B);
    vector<string> OBJA_point_IDs = A.getPointIDs();
    if (common_points_AB.size() < 2) {    // Need 2 common points to be adjacent
        ROS_INFO("Assignment not executable since some areas are not adjacent");
	} else {
		// = find(ismember(OBJA_point_IDs,common_points_AB{1}));
		qmax = OBJA_point_IDs.size();
		for (int q = 0; q < qmax; ++q) {
			if (OBJA_point_IDs[q].compare(common_points_AB[0]) == 0) {
				ind = q;
			}
		}
		rotate(OBJA_point_IDs.begin(), OBJA_point_IDs.begin() + ind, OBJA_point_IDs.end());
		//rotate(OBJA_point_IDs.rbegin(), OBJA_point_IDs.rbegin() -ind, OBJA_point_IDs.rend());
		if (OBJA_point_IDs[1].compare(common_points_AB[0]) == 0 || OBJA_point_IDs[1].compare(common_points_AB[1]) == 0) {
			rotate(OBJA_point_IDs.begin(), OBJA_point_IDs.begin() + 1, OBJA_point_IDs.end());
		}
	}

    vector<string> wallsA = {OBJA_point_IDs[2], OBJA_point_IDs[3], OBJA_point_IDs[0], OBJA_point_IDs[1]};
    
    string B_rb = OBJA_point_IDs[3];
          
    vector<string> common_points_BC = getCommonPoints(B,C);
	vector<string> OBJC_point_IDs = C.getPointIDs();
    if (common_points_BC.size() < 2) {    // Need 2 common points to be adjacent
        ROS_INFO("Assignment not executable since some areas are not adjacent");
    } else {
		// = find(ismember(OBJC_point_IDs,common_points_BC{1}));
		qmax = OBJC_point_IDs.size();
		for (int q = 0; q < qmax; ++q) {
			if (OBJC_point_IDs[q].compare(common_points_BC[0]) == 0) {
				ind = q;
			}
		}
		rotate(OBJC_point_IDs.begin(), OBJC_point_IDs.begin() + ind, OBJC_point_IDs.end());
		//rotate(OBJC_point_IDs.rbegin(), OBJC_point_IDs.rbegin() -ind, OBJC_point_IDs.rend());
		if (OBJC_point_IDs[1].compare(common_points_BC[0]) == 0 || OBJC_point_IDs[1].compare(common_points_BC[1]) == 0) {
			rotate(OBJC_point_IDs.begin(), OBJC_point_IDs.begin() + 1, OBJC_point_IDs.end());
		}
	}

    vector<string> wallsC = {OBJC_point_IDs[2], OBJC_point_IDs[3], OBJC_point_IDs[0], OBJC_point_IDs[1]};
    
	string pivotpoint;
	bool foundpivot = false;
    
	for (int i = 0; i < common_points_AB.size(); ++i) {
		for (int j = 0; j < common_points_BC.size(); ++j) {
			if (common_points_AB[i].compare(common_points_BC[j]) == 0) {
				pivotpoint = common_points_AB[i];
				foundpivot = true;
			}
		}
	}

	vector<string> OBJB_point_IDs = B.getPointIDs();
	qmax = OBJB_point_IDs.size();
	for (int q = 0; q < qmax; ++q) {
		if (OBJB_point_IDs[q].compare(B_rb) == 0) {
			ind = q;
		}
	}
	rotate(OBJB_point_IDs.begin(), OBJB_point_IDs.begin() + ind, OBJB_point_IDs.end());
	//rotate(OBJB_point_IDs.rbegin(), OBJB_point_IDs.rbegin() -ind, OBJB_point_IDs.rend());
    
	vector<string> wallsB;

    if (foundpivot) {
		int index_dir; // index_dir = find(ismember(OBJB_point_IDs,pivotpoint));
		int pmax = OBJB_point_IDs.size();
		for (int p = 0; p < pmax; ++p) {
			if (OBJB_point_IDs[p].compare(pivotpoint) == 0) {
				index_dir = p;
			}
		}

        if (index_dir == 1) {
			wallsB = {OBJB_point_IDs[1], OBJB_point_IDs[2], OBJB_point_IDs[2], OBJB_point_IDs[3]};
		} else if (index_dir == 4) {
			wallsB = {OBJB_point_IDs[0], OBJB_point_IDs[1], OBJB_point_IDs[1], OBJB_point_IDs[2]};
		} else {
            ROS_INFO("Cannot turn around node 2 or 3");
		}
	} else {
		wallsB = {OBJB_point_IDs[0], OBJB_point_IDs[1], OBJB_point_IDs[2], OBJB_point_IDs[3]};
	}
 
    vector<string> wallsABC;
	wallsABC.reserve( wallsA.size() + wallsB.size() + wallsC.size() ); // preallocate memory
	wallsABC.insert( wallsABC.end(), wallsA.begin(), wallsA.end() );
	wallsABC.insert( wallsABC.end(), wallsB.begin(), wallsB.end() );
	wallsABC.insert( wallsABC.end(), wallsC.begin(), wallsC.end() );
	return wallsABC;
}	

double getSteeringTurnSharp(Point ropodpos, double ropod_angle, bool dir_cw, array<string, 6> task, vector<PointID> pointlist) {
    // NOT IN LOCAL COORDINATES (YET)
    // Function to determine steering action while rotating around a point
    // aka taking a turn, this time with a sharp angle.
    // In the special case of a sharp corner we also need to take the wall 
    // from the next hallway into account.
    // dir: direction 0 = CCW, 1 = CW rotation
    PointID obj2wall_p0 = getPointByID(task[0],pointlist);
    PointID obj2wall_p1 = getPointByID(task[1],pointlist);
    PointID obj3wall_p0 = getPointByID(task[2],pointlist);
    PointID obj3wall_p1 = getPointByID(task[3],pointlist);
	Point obj2wall_p0_idless (obj2wall_p0.x, obj2wall_p0.y);
	Point obj3wall_p0_idless (obj3wall_p0.x, obj3wall_p0.y);

    double obj2wall_angle = atan2(obj2wall_p1.y-obj2wall_p0.y, obj2wall_p1.x-obj2wall_p0.x);
    double obj3wall_angle = atan2(obj3wall_p1.y-obj3wall_p0.y, obj3wall_p1.x-obj3wall_p0.x);
    // ROS_INFO("Sharp steering obj2ang: %f / obj3ang: %f", obj2wall_angle, obj3wall_angle);
    PointID pivot = getPointByID(task[4],pointlist);

    // Determine steering around point
    Point local_pivot = coordGlobalToRopod(pivot, ropodpos, ropod_angle);
    double phi_0 = steerAroundPoint(local_pivot, dir_cw); //dir: 0 = CCW, 1 = CW rotation

    double x_rearax = ropodpos.x - D_AX*cos(ropod_angle); // X position of center of rear axle [m]
    double y_rearax = ropodpos.y - D_AX*sin(ropod_angle); // Y position of center of rear axle [m]
	Point rt(x_rearax+(D_AX+SIZE_FRONT_ROPOD)*cos(ropod_angle)+SIZE_SIDE*cos(ropod_angle-M_PI/2),
			 y_rearax+(D_AX+SIZE_FRONT_ROPOD)*sin(ropod_angle)+SIZE_SIDE*sin(ropod_angle-M_PI/2));
	Point lt(x_rearax+(D_AX+SIZE_FRONT_ROPOD)*cos(ropod_angle)+SIZE_SIDE*cos(ropod_angle+M_PI/2),
			 y_rearax+(D_AX+SIZE_FRONT_ROPOD)*sin(ropod_angle)+SIZE_SIDE*sin(ropod_angle+M_PI/2));
    Point fr(FEELER_SIZE_STEERING*cos(ropod_angle+phi_0),FEELER_SIZE_STEERING*sin(ropod_angle+phi_0));
	Point fl(FEELER_SIZE_STEERING*cos(ropod_angle+phi_0),FEELER_SIZE_STEERING*sin(ropod_angle+phi_0));
	fr = fr.add(rt); fl = fl.add(lt);

	Point obj3_wall_to_fl = rotate_point(obj3wall_p0_idless, -obj3wall_angle, fl);
	Point obj2_wall_to_fl = rotate_point(obj2wall_p0_idless, -obj2wall_angle, fl);
	Point obj3_wall_to_fr = rotate_point(obj3wall_p0_idless, -obj3wall_angle, fr);
	Point obj2_wall_to_fr = rotate_point(obj2wall_p0_idless, -obj2wall_angle, fr);
	obj3_wall_to_fl = obj3_wall_to_fl.sub(obj3wall_p0);
	obj2_wall_to_fl = obj2_wall_to_fl.sub(obj2wall_p0);
	obj3_wall_to_fr = obj3_wall_to_fr.sub(obj3wall_p0);
	obj2_wall_to_fr = obj2_wall_to_fr.sub(obj2wall_p0);

	double obj3_wall_to_feeler, obj2_wall_to_feeler;
    // Pick feeler that is closest to wall (and thus most in 'danger')
    if (abs(obj3_wall_to_fl.y) < abs(obj3_wall_to_fr.y)) {
        obj3_wall_to_feeler = obj3_wall_to_fl.y;
	} else {
        obj3_wall_to_feeler = obj3_wall_to_fr.y;
    }
    if (abs(obj2_wall_to_fl.y) < abs(obj2_wall_to_fr.y)) {
        obj2_wall_to_feeler = obj2_wall_to_fl.y;
	} else {
        obj2_wall_to_feeler = obj2_wall_to_fr.y;
	}

	double frac_in_trans;
	Point to_middle_vec, to_front_vec;

    if (task[5].compare("right") == 0) {
        if (obj3_wall_to_feeler > -ENV_TCTW_SIZE) {
            //disp('tctw obj3');
			//Point rotate_point(Point c, double angle, Point p);
            Point obj3_wall_to_ropod = rotate_point(obj3wall_p0_idless, -obj3wall_angle, ropodpos); 			
			obj3_wall_to_ropod = obj3_wall_to_ropod.sub(obj3wall_p0_idless);
            to_middle_vec.x = (-FOLLOW_WALL_DIST_TURNING-obj3_wall_to_ropod.y)*-sin(obj3wall_angle);
			to_middle_vec.y = (-FOLLOW_WALL_DIST_TURNING-obj3_wall_to_ropod.y)*cos(obj3wall_angle);
			to_front_vec.x = CARROT_LENGTH*cos(obj3wall_angle);
            to_front_vec.y = CARROT_LENGTH*sin(obj3wall_angle);
            frac_in_trans = 1;
        } else if (obj2_wall_to_feeler > -ENV_TCTW_SIZE) {
            //disp('tctw obj2');
            Point obj2_wall_to_ropod = rotate_point(obj2wall_p0_idless, -obj2wall_angle, ropodpos); 			
			obj2_wall_to_ropod = obj2_wall_to_ropod.sub(obj2wall_p0_idless);
            to_middle_vec.x = (-FOLLOW_WALL_DIST_TURNING-obj2_wall_to_ropod.y)*-sin(obj2wall_angle);
			to_middle_vec.y = (-FOLLOW_WALL_DIST_TURNING-obj2_wall_to_ropod.y)*cos(obj2wall_angle);
			to_front_vec.x = CARROT_LENGTH*cos(obj2wall_angle);
            to_front_vec.y = CARROT_LENGTH*sin(obj2wall_angle);
            frac_in_trans = 1;
        } else if (obj3_wall_to_feeler > -(ENV_TCTW_SIZE+ENV_TRNS_SIZE_CORNERING)) {
            //disp('trans obj3');
            Point obj3_wall_to_ropod = rotate_point(obj3wall_p0_idless, -obj3wall_angle, ropodpos); 			
			obj3_wall_to_ropod = obj3_wall_to_ropod.sub(obj3wall_p0_idless);
            to_middle_vec.x = (-FOLLOW_WALL_DIST_TURNING-obj3_wall_to_ropod.y)*-sin(obj3wall_angle);
            to_middle_vec.y = (-FOLLOW_WALL_DIST_TURNING-obj3_wall_to_ropod.y)*cos(obj3wall_angle);
            to_front_vec.x = CARROT_LENGTH*cos(obj3wall_angle);
            to_front_vec.y = CARROT_LENGTH*sin(obj3wall_angle);
            frac_in_trans = 1-(-obj3_wall_to_feeler-ENV_TCTW_SIZE)/ENV_TRNS_SIZE_CORNERING;
        } else if (obj2_wall_to_feeler > -(ENV_TCTW_SIZE+ENV_TRNS_SIZE_CORNERING)) {
            //disp('trans obj2');
            Point obj2_wall_to_ropod = rotate_point(obj2wall_p0_idless, -obj2wall_angle, ropodpos); 			 			
			obj2_wall_to_ropod = obj2_wall_to_ropod.sub(obj2wall_p0_idless);
            to_middle_vec.x = (-FOLLOW_WALL_DIST_TURNING-obj2_wall_to_ropod.y)*-sin(obj2wall_angle);
            to_middle_vec.y = (-FOLLOW_WALL_DIST_TURNING-obj2_wall_to_ropod.y)*cos(obj2wall_angle);
            to_front_vec.x = CARROT_LENGTH*cos(obj2wall_angle);
            to_front_vec.y = CARROT_LENGTH*sin(obj2wall_angle);
            frac_in_trans = 1-(-obj2_wall_to_feeler-ENV_TCTW_SIZE)/ENV_TRNS_SIZE_CORNERING;
        } else {
            //disp('normal cornering');
            Point obj3_wall_to_ropod = rotate_point(obj3wall_p0_idless, -obj3wall_angle, ropodpos); 			
			obj3_wall_to_ropod = obj3_wall_to_ropod.sub(obj3wall_p0_idless);
            to_middle_vec.x = (-FOLLOW_WALL_DIST_TURNING-obj3_wall_to_ropod.y)*-sin(obj3wall_angle);
            to_middle_vec.y = (-FOLLOW_WALL_DIST_TURNING-obj3_wall_to_ropod.y)*cos(obj3wall_angle);
            to_front_vec.x = CARROT_LENGTH*cos(obj3wall_angle);
            to_front_vec.y = CARROT_LENGTH*sin(obj3wall_angle);
            frac_in_trans = 0;
		}
	} else {
        if (obj3_wall_to_feeler < ENV_TCTW_SIZE) {
            //disp('tctw obj3');
            Point obj3_wall_to_ropod = rotate_point(obj3wall_p0_idless, -obj3wall_angle, ropodpos); 			
			obj3_wall_to_ropod = obj3_wall_to_ropod.sub(obj3wall_p0_idless);
			to_middle_vec.x = (FOLLOW_WALL_DIST_TURNING-obj3_wall_to_ropod.y)*-sin(obj3wall_angle);
            to_middle_vec.y = (FOLLOW_WALL_DIST_TURNING-obj3_wall_to_ropod.y)*cos(obj3wall_angle);
            to_front_vec.x = CARROT_LENGTH*cos(obj3wall_angle);
            to_front_vec.y = CARROT_LENGTH*sin(obj3wall_angle);
            frac_in_trans = 1;
        } else if (obj2_wall_to_feeler < ENV_TCTW_SIZE) {
            //disp('tctw obj2');
            Point obj2_wall_to_ropod = rotate_point(obj2wall_p0_idless, -obj2wall_angle, ropodpos); 			 			
			obj2_wall_to_ropod = obj2_wall_to_ropod.sub(obj2wall_p0_idless);
            to_middle_vec.x = (FOLLOW_WALL_DIST_TURNING-obj2_wall_to_ropod.y)*-sin(obj2wall_angle);
            to_middle_vec.y = (FOLLOW_WALL_DIST_TURNING-obj2_wall_to_ropod.y)*cos(obj2wall_angle);
            to_front_vec.x = CARROT_LENGTH*cos(obj2wall_angle);
            to_front_vec.y = CARROT_LENGTH*sin(obj2wall_angle);
            frac_in_trans = 1;
        } else if (obj3_wall_to_feeler < (ENV_TCTW_SIZE+ENV_TRNS_SIZE_CORNERING)) {
            //disp('trans obj3');
            Point obj3_wall_to_ropod = rotate_point(obj3wall_p0_idless, -obj3wall_angle, ropodpos); 			
			obj3_wall_to_ropod = obj3_wall_to_ropod.sub(obj3wall_p0_idless);
            to_middle_vec.x = (FOLLOW_WALL_DIST_TURNING-obj3_wall_to_ropod.y)*-sin(obj3wall_angle);
            to_middle_vec.y = (FOLLOW_WALL_DIST_TURNING-obj3_wall_to_ropod.y)*cos(obj3wall_angle);
            to_front_vec.x = CARROT_LENGTH*cos(obj3wall_angle);
            to_front_vec.y = CARROT_LENGTH*sin(obj3wall_angle);
            frac_in_trans = 1-(obj3_wall_to_feeler-ENV_TCTW_SIZE)/ENV_TRNS_SIZE_CORNERING;
        } else if (obj2_wall_to_feeler < (ENV_TCTW_SIZE+ENV_TRNS_SIZE_CORNERING)) {
            //disp('trans obj2');
            Point obj2_wall_to_ropod = rotate_point(obj2wall_p0_idless, -obj2wall_angle, ropodpos); 			 			
			obj2_wall_to_ropod = obj2_wall_to_ropod.sub(obj2wall_p0_idless);
            to_middle_vec.x = (FOLLOW_WALL_DIST_TURNING-obj2_wall_to_ropod.y)*-sin(obj2wall_angle);
            to_middle_vec.y = (FOLLOW_WALL_DIST_TURNING-obj2_wall_to_ropod.y)*cos(obj2wall_angle);
            to_front_vec.x = CARROT_LENGTH*cos(obj2wall_angle);
            to_front_vec.y = CARROT_LENGTH*sin(obj2wall_angle);
            frac_in_trans = 1-(obj2_wall_to_feeler-ENV_TCTW_SIZE)/ENV_TRNS_SIZE_CORNERING;
        } else {
            //disp('normal cornering');
            Point obj3_wall_to_ropod = rotate_point(obj3wall_p0_idless, -obj3wall_angle, ropodpos); 			
			obj3_wall_to_ropod = obj3_wall_to_ropod.sub(obj3wall_p0_idless);
            to_middle_vec.x = (-FOLLOW_WALL_DIST_TURNING-obj3_wall_to_ropod.y)*-sin(obj3wall_angle);
            to_middle_vec.y = (-FOLLOW_WALL_DIST_TURNING-obj3_wall_to_ropod.y)*cos(obj3wall_angle);
            to_front_vec.x = CARROT_LENGTH*cos(obj3wall_angle);
            to_front_vec.y = CARROT_LENGTH*sin(obj3wall_angle);
            frac_in_trans = 0;
		}
	}

	Point tctw_steer_vec = to_middle_vec.add(to_front_vec);
    double phi_tctw = atan2(tctw_steer_vec.y,tctw_steer_vec.x)-ropod_angle;
    return frac_in_trans*phi_tctw+(1-frac_in_trans)*phi_0;
}

// wrap [rad] angle to [-PI..PI)
double wrapToPi(double angle) {
	return modf(angle + _PI, _TWO_PI) - _PI;
}

double modf(double x, double y) {
    //static_assert(!std::numeric_limits<T>::is_exact , "Mod: floating-point type expected");
    if (0. == y)
        return x;
    double m = x - y * floor(x/y);
    // handle boundary cases resulted from floating-point cut off:
    if (y > 0)              // modulo range: [0..y)
    {
        if (m>=y)           // Mod(-1e-16             , 360.    ): m= 360.
            return 0;
        if (m<0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(106.81415022205296 , _TWO_PI ): m= -1.421e-14 
        }
    }
    else                    // modulo range: (y..0]
    {
        if (m<=y)           // Mod(1e-16              , -360.   ): m= -360.
            return 0;
        if (m>0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14 
        }
    }
    return m;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

string get_date(void) {
   time_t now;
   char the_date[MAX_DATE];

   the_date[0] = '\0';

   now = time(NULL);

   if (now != -1)
   {
      strftime(the_date, MAX_DATE, "%m_%d-%H_%M", gmtime(&now));
   }

   return string(the_date);
}