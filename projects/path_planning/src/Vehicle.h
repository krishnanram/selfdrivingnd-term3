#ifndef VEHICLE_H
#define VEHICLE_H
#include <math.h>
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "CostProvider.h"
#include "Snapshot.h"

using namespace std;

class Vehicle {

public:

    double target_speed;
    string state;
    int lane;
    double s;
    double v;
    double a;
    int in_front;   // number of vehicles "in front" of my-self-driving-car
    int at_behind;  // number of vehicles "behind" my-self-driving-car

    Vehicle(int lane, double s, double v, double a = 0, bool my_self_driving_car = false);
    virtual ~Vehicle();

    void update_state(map<int, vector <vector<double>>> predictions);
    void increment(double dt = 1.0, bool overide = false);
    void realize_state(map<int, vector<vector<double>>> predictions);
    vector<vector<double>> generate_predictions(int horizon = 5);


private:

    CostProvider cost;
    struct snapshot {
        int lane;
        double s;
        double v;
        double a;
        string state;
    };

    int lanes_available;
    bool my_self_driving_car;
    double max_acceleration;
    double max_decceleration;


    string get_next_state(map<int, vector <vector<double>>> predictions);
    void realize_prep_lane_change(map<int, vector<vector<double>>> predictions, string direction);
    void realize_lane_change(map<int, vector<vector<double>>> predictions, string direction);
    void realize_keep_lane(map<int, vector<vector<double>>> predictions);
    double _max_accel_for_lane(map<int, vector<vector<double>>> predictions, int lane, double s);
    void realize_constant_speed();
    vector<double> state_at(double t);
    vector<Snapshot> trajectories_for_state(string state, map<int, vector <vector<double>>> predictions, int horizon = 5);
    string min_cost_state(vector<string> states, vector<double> costs);
};
#endif /* VEHICLE_H */
