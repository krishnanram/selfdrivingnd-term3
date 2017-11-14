#ifndef COST_H
#define COST_H
#include <math.h>
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "Snapshot.h"

using namespace std;

struct TrajectoryData {
    int proposed_lane;
    double avg_speed;
    double max_acceleration;
    double closest_approach;
    bool collides;
    double collides_at;
};

class Vehicle;

class CostProvider {


public:

    CostProvider();
    virtual ~CostProvider();

    double calculate_cost(const Vehicle &vehicle, vector<Snapshot> trajectories, map<int, vector<vector<double>>> predictions);


private :

    TrajectoryData get_trajectory_data(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<double>>> predictions);

    map<int, vector<vector<double>>> filter_predictions_by_lane(map<int, vector<vector<double>>> predictions, int lane);
    bool check_collision(Snapshot snapshot, double s_prev, double s_curr);

    double inefficiency_cost(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<double>>> predictions, TrajectoryData data);

    double collision_cost(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<double>>> predictions, TrajectoryData data);

    double buffer_cost(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<double>>> predictions, TrajectoryData data);
};


#endif /* COST_H */
