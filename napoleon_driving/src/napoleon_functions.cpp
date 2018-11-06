#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
using namespace std;
#include "napoleon_config.h"
#include "napoleon_functions.h"
#include <ros/ros.h>

Point::Point(double xval = 0.0, double yval = 0.0) {
	x = xval;
	y = yval;
}

Point Point::sub(Point b) {
	return Point(x - b.x, y - b.y);
}

Point rotate_point(double cx, double cy, double angle, Point p)
{
	double s = sin(angle);
	double c = cos(angle);

	// translate point back to origin:
	p.x -= cx;
	p.y -= cy;

	// rotate point
	double xnew = p.x * c - p.y * s;
	double ynew = p.x * s + p.y * c;

	// translate point back:
	p.x = xnew + cx;
	p.y = ynew + cy;
	return p;
}


Point coordGlobalToRopod(Point gp, Point rc, double ra)
{
	// Function that transforms a point from the global coordinate system to the
	// coordinate system of the ropod[X up(pos), Y left(pos)].
	// gp global point[x, y]
	// rc ropod center[x, y]
	// ra ropod angle in global coordinate system[rad]
	Point vec_glob = gp.sub(rc); // vector from ropod to point in global coord
	Point P = rotate_point(0, 0, -ra, vec_glob); // [X, Y] position in ropod coordinates
	//cout << "In func coordGlobaltoRopod: " << P.x << ", " << P.y << endl;
	// X up, Y left, theta CCW, theta = 0 at X - axis
	return P;
}

double getSteering(Point local_wallpoint_front, Point local_wallpoint_rear)
{
	// Function to determine steering action while cruising
	// ropod_length the length of the ropod[m]
	// size_side is the distance from the center to the side of the vehicle[m]
	// feeler_size is the length of the feeler, extending in front of the ropod
	// and triggering behavior changes - positioned above tr(top right) and tl[m]
	// local_wallpoint_front: point on right wall is a point that is to the
	// right of the ropod, can be any point(x, y) format[m]
	// local_wallpoint_rear : has to lie behind local_wallpoint_front, behind
	// as in relative to the direction in which the ropod wants to go
	// ropod_phi : current steering action[rad]
	// env_cor_width is the total corridor width[m]
	// env_tctw_size is the distance when the ropod is too close to the wall[m]
	// env_trns_size is the size of the transition area(starting from tctw)[m]
	// env_carrot_size is how aggressive the ropod steers to the middle, it is
	// the scale for the vector that points forward from the middle of the road,
	// the bigger this value, the less aggressive the steering action[m]
	// fimdf: feelers in motion direction fraction, this determines whether
	// the feelers are pointed in the same direction as the wheels[1] or
	// just in front of the ropod[0] or something in between[0, 1]
	if (env_cor_width / 2 < 2 * (size_side + env_tctw_size + env_trns_size))
	{
		cout << "Corridor too small, or transition and too close too wall zone too big" << endl;
	}

	// Feelers in front of the ropod
	Point local_fl(ropod_length / 2 + feeler_size, size_side);
	Point local_fr(ropod_length / 2 + feeler_size, -size_side);

	double local_wall_angle = atan2(local_wallpoint_front.y - local_wallpoint_rear.y, local_wallpoint_front.x - local_wallpoint_rear.x);

	Point fl_env_0, rm_env_0, fr_env_0, rb_env_0;

	fl_env_0 = rotate_point(0, 0, -local_wall_angle, local_fl);
	//fl_env_0 = rotatepoint(local_fl, zerovec, -local_wall_angle);               // Feeler left @ env at theta = 0
	//rm_env_0 = rotatepoint(local_wallpoint_front, zerovec, -local_wall_angle);	// Middle of lane @ env at theta = 0
	rm_env_0 = rotate_point(0, 0, -local_wall_angle, local_wallpoint_front);
	rm_env_0.y += (env_cor_width / 2);
	double dist_left = rm_env_0.y - fl_env_0.y;

	//fr_env_0 = rotatepoint(local_fr, zerovec, -local_wall_angle);				// Feeler right @ env at theta = 0
	fr_env_0 = rotate_point(0, 0, -local_wall_angle, local_fr);
	//rb_env_0 = rotatepoint(local_wallpoint_rear, zerovec, -local_wall_angle);	// Wall at right side @ env at theta = 0
	rb_env_0 = rotate_point(0, 0, -local_wall_angle, local_wallpoint_rear);
	double dist_right = fr_env_0.y - rb_env_0.y;								// Y distance from feeler right to the wall(neg if beyond wall)

	double dist_ropod_center_to_rightwall = -rb_env_0.y;						// Y distance from ropod center to right wall

	double to_middle_size = env_cor_width / 4 - dist_ropod_center_to_rightwall;					// Y Distance from ropod center to middle of road @ env at theta = 0
	double to_middle_vec[2] = { to_middle_size * -sin(local_wall_angle), to_middle_size * cos(local_wall_angle) };	// Vector from ropod center to middle of road
	double to_front_vec[2] = { env_carrot_size * cos(local_wall_angle), env_carrot_size*sin(local_wall_angle) };	// Vector from middle of road to a point further on the road
	double steer_vec[2] = { to_front_vec[0] + to_middle_vec[0], to_front_vec[1] + to_middle_vec[1] };               // Together, these vectors form the steering vector

	// Phi_0 can be used when we are comfortably within the middle of the road
	// Phi_tctw will be used when the feelers are really close to the wall
	// In the transition zone we will do a combination of Phi_0 and Phi_tctw
	double phi_0 = 0;
	double phi_tctw = atan2(steer_vec[1], steer_vec[0]); // Determine the angle of this steering vector
	double phi = phi_0;

	if (dist_right < env_tctw_size && dist_left < env_tctw_size)
	{
		//warning('Poor condition: ropod is too close to left AND right wall');
		phi = phi_tctw;
	}
	else if (dist_right < env_tctw_size)
	{
		// disp('Ropod is too close to right side, steer back to middle');
		phi = phi_tctw;
	}
	else if (dist_left < env_tctw_size)
	{
		// disp('Ropod is too close to left side, steer back to middle');
		phi = phi_tctw;
	}
	else if (dist_right < env_tctw_size + env_trns_size && dist_left < env_tctw_size + env_trns_size)
	{
		//warning('Poor condition: ropod is in transition zone on both sides');
		phi = phi_tctw;
	}
	else if (dist_right < env_tctw_size + env_trns_size)
	{
		// disp('Ropod is in transition zone on the right side');
		double frac_in_trans = 1 - (dist_right - env_tctw_size) / env_trns_size;
		phi = frac_in_trans * phi_tctw + (1 - frac_in_trans)*phi_0;
	}
	else if (dist_left < env_tctw_size + env_trns_size)
	{
		// disp('Ropod is in transition zone on the left side');
		double frac_in_trans = 1 - (dist_left - env_tctw_size) / env_trns_size;
		phi = frac_in_trans * phi_tctw + (1 - frac_in_trans)*phi_0;
	}
	else
	{
		// disp('Ropod is comfortably in the middle of the lane, no steering action necessary');
		phi = phi_0;
	}

	return phi;
}