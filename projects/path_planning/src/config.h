#ifndef CONFIG_H
#define CONFIG_H

using namespace std;

const bool DEBUG = true;

const double MPH_TO_MPS = 2.24;     // MPH to meters-per-second, 2.24mph -> 1 m/s
const double SPLINE_SPACING = 30;   // 30 meters between spline segments
const int MAX_TICKS = 30;           // number of simulator "ticks" before behavior planner processing for ego
const double SECS_PER_TICK = 0.02;  // number of elapsed seconds per "tick" of simulator
const int FILLER_DIST = 49.05;         // 50 meters


const int    MY_SELF_DRIVING_CAR_START_LANE = 1;       // lane where my_self_driving_car  starts in (fixed in simulator)
const double MY_SELF_DRIVING_CAR_MAX_ACCEL = 22.0;  // 22 meters-per-second acceleration value
const double MY_SELF_DRIVING_CAR_MAX_DECEL = 11.0;  // 11 meters-per-second decceleration value
const double MY_SELF_DRIVING_CAR_MAX_VELOCITY = 50.00;

const int NUM_LANES = 3;

const double PREFERRED_BUFFER = 25;  // 0.5s buffer (25 * 0.02) - impacts "keep lane" behavior.

const int PREDICTIONS_HZ = 75;  // 1.5s horizon (75 * 0.02)
const int TRAJECTORIES_HZ = 75;

const double LEADING_HZ = 150.0;  // how far out directly ahead of us to find the closest car (3s horizon - 150 * 0.02)
const double FOLLOW_HZ = 175.0;   // how far out behind us to find the closest car (3.5s horizon - 175 * 0.02)


const double COLLISION  = pow(10.0, 7);
const double DANGER     = pow(10.0, 6);
const double EFFICIENCY = pow(10.5, 4);  // prioritize efficiency just slightly over comfort
const double COMFORT    = pow(10.0, 4);

#endif /* CONFIG_H */
