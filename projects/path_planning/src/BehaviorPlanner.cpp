#include <algorithm>

#include "BehaviorPlanner.h"
#include "Vehicle.h"


BehaviorPlanner::BehaviorPlanner(Vehicle &car) : my_self_driving_car_car (car) {
    proposed_lane = car.lane;
};

BehaviorPlanner::~BehaviorPlanner() {};

void BehaviorPlanner::update_state(map<int, vector<prediction>> predictions) {

    cout << "Inside BehaviorPlanner:update_state ...\n";
    /*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
    - The vehicle will attempt to drive its target speed, unless there is
    traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
    - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
    behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
    - The vehicle will find the nearest vehicle in the adjacent lane which is
    BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
    3 : [
    {"s" : 4, "lane": 0},
    {"s" : 6, "lane": 0},
    {"s" : 8, "lane": 0},
    {"s" : 10, "lane": 0},
    ]
    }

    */
    state = get_next_state (predictions);
}

CarState BehaviorPlanner::get_next_state(map<int, vector<prediction>> predictions) {
    vector <CarState> states;
    if (state == CarState::PLCR) {
        states = vector < CarState > {CarState::KL, CarState::PLCR, CarState::LCR};
    } else if (state == CarState::PLCL) {
        states = vector < CarState > {CarState::KL, CarState::PLCL, CarState::LCL};
    } else if (state == CarState::LCR) {
        states = vector < CarState > {CarState::KL};
    } else if (state == CarState::LCL) {
        states = vector < CarState > {CarState::KL};
    } else {
        states = vector < CarState > {CarState::KL};
        if (my_self_driving_car_car.lane > 0) {
            states.push_back (CarState::PLCL);
        }
        if (my_self_driving_car_car.lane < lanes_available - 1) {
            states.push_back (CarState::PLCR);
        }
    }

    if (states.size () == 1) {
        return states[0];
    }
    auto costs = vector<estimate> ();
    for (auto state : states) {
        estimate estimate;
        estimate.state = state;
        auto trajectory = trajectory_for_state (state, predictions, PREDICTIONS_COUNT);
        estimate.cost = costProvider.calculate_cost (car_s, ref_vel, trajectory, predictions, state);
        costs.push_back (estimate);
    }
    auto best = min_element (std::begin (costs), std::end (costs),
                             [](estimate est1, estimate est2) {
                                 return est1.cost < est2.cost;
                             });

    if (verbosity) {
        cout << "best estimate: " << (*best).cost << " in state " << as_integer ((*best).state)
             << " in lane: " << my_self_driving_car_car.lane << endl;
    }
    return (*best).state;
}

vector <snapshot>
BehaviorPlanner::trajectory_for_state(CarState proposed_state, map<int, vector<prediction>> predictions, int horizon) {

    cout << "Inside BehaviorPlanner:trajectory_for_state ...\n";
    // remember current state
    auto initial_snapshot = get_snapshot ();

    // pretend to be in new proposed state
    //state = proposed_state;
    vector <snapshot> trajectory = {initial_snapshot};
    for (int i = 0; i < horizon; ++i) {
        restore_state_from_snapshot (initial_snapshot);
        state = proposed_state;
        realize_state (predictions);
        my_self_driving_car_car.increment (i * PREDICTION_INTERVAL);
        trajectory.push_back (get_snapshot ());

        // need to remove first prediction for each vehicle.
        for (auto pair : predictions) {
            auto vehicle_pred = pair.second;
            vehicle_pred.erase (vehicle_pred.begin ());
        }
    }

    // restore state from snapshot
    restore_state_from_snapshot (initial_snapshot);
    return trajectory;
}

void BehaviorPlanner::realize_state(map<int, vector<prediction>> predictions) {

    cout << "Inside BehaviorPlanner:realize_state ...\n";
    //Given a state, realize it by adjusting velocity and lane.
    if (state == CarState::CS) {
        realize_constant_speed ();
    } else if (state == CarState::KL) {
        realize_keep_lane (predictions);
    } else if (state == CarState::LCL) {
        realize_lane_change (predictions, "L");
    } else if (state == CarState::LCR) {
        realize_lane_change (predictions, "R");
    } else if (state == CarState::PLCL) {
        realize_prep_lane_change (predictions, "L");
    } else if (state == CarState::PLCR) {
        realize_prep_lane_change (predictions, "R");
    }
}

void BehaviorPlanner::realize_constant_speed() {}

void BehaviorPlanner::_update_ref_speed_for_lane(map<int, vector<prediction>> predictions, int checked_lane) {

    cout << "Inside BehaviorPlanner:_update_ref_speed_for_lane ...\n";
    bool too_close = false, keep_speed = false, danger = false;
    double max_speed = MAX_SPEED;

    for (auto pair : predictions) {
        prediction pred = pair.second[0];
        double target_speed = pred.get_velocity ();
        if (my_self_driving_car_car.is_behind_of (pred, checked_lane) && target_speed < max_speed) {
            // follow the car behavior
            max_speed = target_speed - SPEED_INCREMENT;
            keep_speed = true;
        }
        if (my_self_driving_car_car.is_close_to (pred, checked_lane)) {

            if (verbosity) {
                cout << "pred d: " << pred.d << " my lane: " << checked_lane;
                cout << " pred s: " << pred.s << " my s: " << my_self_driving_car_car.s << endl;
            }
            if (pred.s < my_self_driving_car_car.s + 5) {
                danger = true;
            }
            too_close = true;
        }
    }
    double velocity = ref_vel;
    if (too_close) {
        if (danger) {
            if (velocity > 40.0) {
                velocity -= 2 * SPEED_INCREMENT;
            } else {
                velocity -= SPEED_INCREMENT;
            }
        } else {
            if (velocity < max_speed) {
                velocity += SPEED_INCREMENT;
            } else if (velocity > max_speed) {
                if (velocity > 40.0) {
                    velocity -= 2 * SPEED_INCREMENT;
                } else {
                    velocity -= SPEED_INCREMENT;
                }
            }
        }
    } else {
        if (keep_speed && velocity > 25 && velocity > max_speed * 2.7) {
            velocity -= SPEED_INCREMENT;
        } else {
            velocity += SPEED_INCREMENT;
        }
        if (velocity > MAX_SPEED) {
            velocity = MAX_SPEED;
        }
    }

    if (velocity < 0) {
        velocity = 0;
    }
    ref_vel = velocity;
}

void BehaviorPlanner::realize_keep_lane(map<int, vector<prediction>> predictions) {

    cout << "Inside BehaviorPlanner:realize_keep_lane ...\n";
    proposed_lane = my_self_driving_car_car.lane;
    if (verbosity) {
        cout << "keep lane: " << my_self_driving_car_car.lane << " proposed lane: " << my_self_driving_car_car.lane << endl;
    }
    _update_ref_speed_for_lane (predictions, my_self_driving_car_car.lane);
}

void BehaviorPlanner::realize_lane_change(map<int, vector<prediction>> predictions, string direction) {

    cout << "Inside BehaviorPlanner:realize_lane_change ...\n";
    int delta = -1;
    if (direction.compare ("R") == 0) {
        delta = 1;
    }
    my_self_driving_car_car.lane += delta;
    proposed_lane = my_self_driving_car_car.lane;
    if (verbosity) {
        cout << "lane change: " << my_self_driving_car_car.lane << " proposed lane: " << proposed_lane << endl;
    }
    _update_ref_speed_for_lane (predictions, proposed_lane);
}

void BehaviorPlanner::realize_prep_lane_change(map<int, vector<prediction>> predictions, string direction) {

    cout << "Inside BehaviorPlanner:realize_prep_lane_change ...\n";
    int delta = -1;
    bool close = false;
    if (direction.compare ("R") == 0) {
        delta = 1;
    }
    proposed_lane = my_self_driving_car_car.lane + delta;
    if (verbosity) {
        cout << "prep lane change: " << my_self_driving_car_car.lane << " proposed lane: " << proposed_lane << endl;
    }

    vector <vector<prediction>> at_behind;
    for (auto pair : predictions) {
        int v_id = pair.first;
        vector <prediction> v = pair.second;
        if (my_self_driving_car_car.is_in_front_of (v[0], proposed_lane)) {
            at_behind.push_back (v);
        }
        if (my_self_driving_car_car.is_close_to (v[0], my_self_driving_car_car.lane)) {
            if (v[0].get_distance (my_self_driving_car_car.x, my_self_driving_car_car.y, my_self_driving_car_car.s) < 4) {
                close = true;
            }
        }
    }
    if (at_behind.size () > 0) {
        double velocity = ref_vel;
        if (close) {
            if (velocity > 40.0) {
                velocity -= 2 * SPEED_INCREMENT;
            } else {
                velocity -= SPEED_INCREMENT;
            }
        } else {
            velocity += SPEED_INCREMENT;
        }
        if (velocity > MAX_SPEED) {
            velocity = MAX_SPEED;
        }
        ref_vel = velocity;
    }
}

double BehaviorPlanner::get_expected_velocity() {
    return ref_vel;
}

void BehaviorPlanner::restore_state_from_snapshot(snapshot snapshot) {

    cout << "Inside BehaviorPlanner:restore_state_from_snapshot ...\n";
    this->my_self_driving_car_car.s = snapshot.s;
    this->my_self_driving_car_car.d = snapshot.d;
    this->my_self_driving_car_car.x = snapshot.x;
    this->my_self_driving_car_car.y = snapshot.y;
    this->my_self_driving_car_car.dx = snapshot.dx;
    this->my_self_driving_car_car.dy = snapshot.dy;
    this->my_self_driving_car_car.ddx = snapshot.ddx;
    this->my_self_driving_car_car.ddy = snapshot.ddy;
    this->my_self_driving_car_car.yaw = snapshot.yaw;
    this->my_self_driving_car_car.lane = snapshot.lane;
    this->state = snapshot.state;
    this->ref_vel = snapshot.ref_vel;
    this->proposed_lane = snapshot.proposed_lane;
}

snapshot BehaviorPlanner::get_snapshot() {

    cout << "Inside BehaviorPlanner:get_snapshot ...\n";
    snapshot snapshot_temp;
    snapshot_temp.x = this->my_self_driving_car_car.x;
    snapshot_temp.y = this->my_self_driving_car_car.y;
    snapshot_temp.dx = this->my_self_driving_car_car.dx;
    snapshot_temp.dy = this->my_self_driving_car_car.dy;
    snapshot_temp.s = this->my_self_driving_car_car.s;
    snapshot_temp.d = this->my_self_driving_car_car.d;
    snapshot_temp.ddx = this->my_self_driving_car_car.ddx;
    snapshot_temp.ddy = this->my_self_driving_car_car.ddy;
    snapshot_temp.yaw = this->my_self_driving_car_car.yaw;
    snapshot_temp.lane = this->my_self_driving_car_car.lane;
    snapshot_temp.state = this->state;
    snapshot_temp.ref_vel = this->ref_vel;
    snapshot_temp.proposed_lane = this->proposed_lane;

    return snapshot_temp;
}

