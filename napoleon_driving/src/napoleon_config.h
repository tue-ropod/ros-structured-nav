#ifndef NAP_CONF
#define NAP_CONF

#define _USE_MATH_DEFINES
#include <math.h>
using namespace std;

// Define environment
const double env_cor_width = 2.45;

// Optimization / performance parameters
const double v_cruising = 1.4;          // Max velocity [m/s] while cruising
const double feeler_size = 0.8;         // Size of feeler [m] - used to predict where ropod goes suppose it would go straight
const double env_tctw_size = 0.05;      // Too close too wall area size [m]
const double env_trns_size = 0.15;      // Transition area size [m]
const double env_carrot_size = 1.50;    // How far ahead point lies where ropod steers towards when too close to a wall [m]

// Vehicle size & size of vectors
const double size_side = 0.72 / 2;      // How wide vehicle is from center
const double ropod_length = 0.35;	    // Ropod length [m]
const double d_ax = 1.1;                // Length from rear axle to ropod center [m]

#endif