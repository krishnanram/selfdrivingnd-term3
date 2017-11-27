#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <chrono>
#include "json.hpp"

#include <iostream>
#include <math.h>
#include <map>
#include <vector>

#include "Vehicle.h"
#include "BehaviorPlanner.h"
#include "Trajectory.h"


using namespace std;
using namespace std::chrono;
using json = nlohmann::json;

class PathPlanner {

public:

    PathPlanner();

    virtual ~PathPlanner();

    void update_vehicle_state(json sensor_fusion);

    void update_my_self_driving_car_car_state(double car_s, double x, double y, double yaw, double s, double d, double speed);

    void generate_trajectory(vector<double> previous_path_x, vector<double> previous_path_y);

    vector<double> get_x_values();

    vector<double> get_y_values();

private:
    double original_yaw;
    double diff;
    milliseconds ms;
    map<int, Vehicle *> vehicles;
    map<int, vector<prediction>> predictions;
    Vehicle my_self_driving_car_car = Vehicle (-1);
    BehaviorPlanner behaviorPlanner = BehaviorPlanner (my_self_driving_car_car);
    Trajectory trajectory = Trajectory ();

    double get_time_step();

};

#endif