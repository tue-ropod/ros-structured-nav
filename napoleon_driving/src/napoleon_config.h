#ifndef NAP_CONF
#define NAP_CONF

#define _USE_MATH_DEFINES
#include <math.h>
using namespace std;

// Define environment
// static constexpr double TUBE_WIDTH = 2.45;

// Optimization / performance parameters
static constexpr double FEELER_SIZE = 0.5;          // Size of feeler [m] - used to predict where ropod goes suppose it would go straight
static constexpr double FEELER_SIZE_STEERING = 0.3; // Size of feeler when steering [m]
static constexpr double ENV_TCTW_SIZE = 0.05;       // Too close too wall area size [m]
static constexpr double ENV_TRNS_SIZE = 0.20;       // Transition area size [m]
static constexpr double CARROT_LENGTH = 1.50;       // How far ahead point lies where ropod steers towards when too close to a wall [m]
static constexpr double ENV_TRNS_SIZE_CORNERING = 0.35; // Transition area size while cornering [m]

// Vehicle size & size of vectors
static constexpr double SIZE_SIDE = 0.72 / 2;      // How wide vehicle is from center [m]
static constexpr double SIZE_FRONT_ROPOD = 0.325;  // How long ropod is from center [m]
static constexpr double ROPOD_LENGTH = 0.65;	   // Ropod length [m]
static constexpr double D_AX = 1.045;              // Length from rear axle to ropod center [m]
static constexpr double FOLLOW_WALL_DIST_TURNING = sqrt(ROPOD_LENGTH*ROPOD_LENGTH/2)+ENV_TCTW_SIZE+ENV_TRNS_SIZE;
static constexpr double SIZE_REAR = 0.10;           // How far vehicle extends behind rear axle
static constexpr double SIZE_FRONT_RAX = D_AX+0.325;// How far vehicle extends in front of rear axle

// Resolutions
static constexpr int F_PLAN = 10;    // Frequency of planning the motion [Hz]
static constexpr int F_FSTR = 10;   // How many times faster the simulation is than the planning [-]
static constexpr int F_MODEL = F_FSTR*F_PLAN;   // Frequency of simulation [Hz]
static constexpr double TS = 1/(double)F_MODEL;         // Sample time of model [s]
// Note: in C++ the resolutions are static, however in Matlab they are dynamic
// When C++ implementation desires dynamic resolutions, declare these elsewhere and not as statics

// Optimization/performance parameters
static constexpr double T_MIN_PRED = 10;            // Predict for n seconds (unless we want to predict an area) [s]
static constexpr double T_PRED_WALL_COLLISION = 05; // Predict for n seconds if ropod collides with walls [s]
static constexpr double T_PRED_OBS_COLLISION = 04;  // Predict for n seconds if ropod collides with obstacles [s]
static constexpr double T_MAX_PRED = 20;            // Predict for n seconds max [s]
static constexpr double DELTA_DOT_LIMIT = M_PI/2;   // Max steering rate per second [rad/s]
static constexpr double CUTOFF_FREQ = 1.0;          // Cutoff frequency for low pass filter to simulate steering delay [Hz]
static constexpr double V_CRUISING = 1.4;           // Max velocity [m/s] while cruising
static constexpr double V_INTER_TURNING = 0.5;      // Max velocity [m/s] when taking a turn
static constexpr double V_INTER_ACC = 0.7;          // Max velocity [m/s] when driving straight at intersection
static constexpr double V_INTER_DEC = 0.3;          // Max velocity [m/s] when driving straight at intersection
static constexpr double V_ENTRY = 0.5;              // Max velocity [m/s] when at entry of intersection
static constexpr double V_STEERSATURATION = 0.2;    // Velocity during steering saturation [m/s]
static constexpr double V_OVERTAKE = 0.5;           // Velocity during overtaking [m/s]
vector<double> V_SCALE_OPTIONS = {1.0, 0.67, 0.33, 0.0};  // Options to scale velocity with
int MAX_K = V_SCALE_OPTIONS.size();                 // Static not happy when populated this way, so nonstatic definition
static constexpr double ENV_COR_WIDTH = 3.00;       
// Fictional hallway width. Ropod will work with lanes of this/2 [m], starting from the wall. 
// No matter what the real hallway size is. This way it will stay close to the right wall, but not too agressively.
static constexpr double TUBE_WIDTH_C = ENV_COR_WIDTH/2; // Default tube width [m]
static constexpr double REACHEDTARGETTRESHOLD = 1.0;  // When x [m] removed from center of last hallway, program finished

// Performance based on position in environment
static constexpr double START_STEERING_EARLY = 0.1;     // Start steering earlier by x [m]
static constexpr double ENTRY_LENGTH = 2.2;             // Length of entries before intersections [m]
static constexpr double ROTATED_ENOUGH_TRES = M_PI/8;   // Stop turning when within x rad of the new corridor
static constexpr double OBS_AVOID_MARGIN = 0.2;         // Margin between ropod and obstacles at full speed [m]
static constexpr double DILATE_ROPOD_ALIGNING = 0.90;   // Dilation from center (so actually this value -size_side if measured from side of vehicle)
//static constexpr double ROPOD_TO_OBS_MARGIN = (OBS_AVOID_MARGIN+SIZE_FRONT_ROPOD)/SIZE_FRONT_ROPOD;
static constexpr double SHARP_ANGLE_TRESHOLD = 0.1;     // Angle how much the next hallway must be sharper than perpendicular to be considered sharp [rad]

// Limits
static constexpr double A_MAX = 0.5;                    // Maximum acceleration magnitude [m/s^2]

// Obstacle
static constexpr double V_OBS_OVERTAKE_MAX = 0.1;       // Max speed an obstacle can have to overtake is [m/s]
static constexpr double MIN_DIST_TO_OVERTAKE = 2.5;     // Don't start earlier than x meters to overtake [m]

#endif
