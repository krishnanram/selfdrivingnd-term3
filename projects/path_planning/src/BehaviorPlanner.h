#ifndef FSM_H
#define FSM_H

#include <iostream>
#include <vector>
#include <map>
#include "CostProvider.h"
#include "Vehicle.h"


using namespace std;

struct estimate {
    CarState state;
    double cost;
};

template<typename Enumeration>
auto as_integer(Enumeration const value)
-> typename std::underlying_type<Enumeration>::type {
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

class BehaviorPlanner {

public:
    BehaviorPlanner(Vehicle &car);

    ~BehaviorPlanner();

    Vehicle &my_self_driving_car_car;
    double car_s;
    double const PREDICTION_INTERVAL = 0.15;

    bool verbosity = false;

    double get_expected_velocity();

    void update_state(map<int, vector<prediction>> predictions);

    void realize_state(map<int, vector<prediction> > predictions);

private:
    double const SPEED_INCREMENT = .224;
    double const MAX_SPEED = 49.50;
    double const TIME_INTERVAL = 0.02;
    double const PREDICTIONS_COUNT = 5;

    int const lanes_available = 3;

    double ref_vel = 0;
    int proposed_lane;
    CarState state = CarState::CS;

    CostProvider costProvider = CostProvider (MAX_SPEED, false);

    void realize_constant_speed();

    void realize_keep_lane(map<int, vector<prediction> > predictions);

    void realize_lane_change(map<int, vector<prediction> > predictions, string direction);

    void realize_prep_lane_change(map<int, vector<prediction> > predictions, string direction);

    CarState get_next_state(map<int, vector<prediction>> predictions);

    void _update_ref_speed_for_lane(map<int, vector<prediction> > predictions, int lane);

    vector<snapshot> trajectory_for_state(CarState state, map<int, vector<prediction>> predictions, int horizon);

    void restore_state_from_snapshot(snapshot snapshot);

    snapshot get_snapshot();
};


#endif
