#include "CostProvider.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>
#include <functional>
#include <numeric>


using namespace std;

CostProvider::CostProvider(double max_speed, bool verbose) {
    MAX_SPEED = max_speed;
    this->verbose = verbose;
}

CostProvider::~CostProvider() {}

double CostProvider::change_lane_cost(vector<snapshot> trajectory,
                                   map<int, vector<prediction>> predictions, TrajectoryData data) const {

    cout << "Inside CostProvider:change_lane_cost ...\n";
    if (data.proposed_lane != data.current_lane) {
        if (data.proposed_lane == 1) {
            return 0;
        }
        return COMFORT;
    }

    return 0;
}

double CostProvider::inefficiency_cost(vector<snapshot> trajectory,
                                    map<int, vector<prediction>> predictions, TrajectoryData data) const {

    cout << "Inside CostProvider:inefficiency_cost ...\n";
    double speed = data.avg_speed;
    double target_speed = MAX_SPEED;
    double diff = target_speed - speed;
    double pct = diff / target_speed;
    double multiplier = pow (pct, 2);
    return 8 * multiplier * EFFICIENCY;
}

double CostProvider::collision_cost(vector<snapshot> trajectory,
                                 map<int, vector<prediction>> predictions, TrajectoryData data) const {

    cout << "Inside CostProvider:collision_cost ...\n";
    if (data.collides.hasCollision) {
        double time_til_collision = 0;
        double exponent = time_til_collision * time_til_collision;
        double mult = exp (-exponent);
        if (verbose) {
            cout << " collision: " << mult * COLLISION << " on step: " << data.collides.step << endl;
        }
        return mult * COLLISION;
    }
    return 0;
}

double CostProvider::free_line_cost(vector<snapshot> trajectory,
                                 map<int, vector<prediction>> predictions, TrajectoryData data) const {

    cout << "Inside CostProvider:free_line_cost ...\n";
    double closest = data.prop_closest_approach;
    if (verbose) {
        cout << "prop closest " << closest << endl;
    }
    if (closest > OBSERVED_DISTANCE) {
        double multiplier = (MAX_DISTANCE - closest) / MAX_DISTANCE;
        return 20 * multiplier * multiplier;
    }
    double multiplier = (OBSERVED_DISTANCE - closest) / OBSERVED_DISTANCE;
    return 5 * multiplier * multiplier * COMFORT;
}

double CostProvider::buffer_cost(vector<snapshot> trajectory,
                              map<int, vector<prediction>> predictions, TrajectoryData data) const {

    cout << "Inside CostProvider:buffer_cost ...\n";
    double closest = data.actual_closest_approach;
    if (verbose) {
        cout << "actual closest " << closest << endl;
    }
    if (closest < Vehicle::SAFE_DISTANCE / 2) {
        return 3 * DANGER;
    }

    if (closest > DESIRED_BUFFER) {
        return 0.0;
    }

    double multiplier = 1.0 - pow ((closest / DESIRED_BUFFER), 2);
    return 3 * multiplier * DANGER;
}

double CostProvider::calculate_cost(double car_s, double ref_vel, vector<snapshot> trajectory,
                                 map<int, vector<prediction>> predictions, CarState state) {

    TrajectoryData trajectory_data = get_helper_data (car_s, ref_vel, trajectory, predictions, state);

    cout << "Inside CostProvider:calculate_cost ...\n";
    double cost = 0.0;
    for (auto cf : delegates) {
        double new_cost = cf (*this, trajectory, predictions, trajectory_data);
        cost += new_cost;
    }
    if (verbose) {
        cout << "has cost " << cost << " for state " << (int) state << endl;
    }
    return cost;
}


CostProvider::TrajectoryData CostProvider::get_helper_data(double car_s, double ref_s, vector<snapshot> trajectory,
                                                     map<int, vector<prediction>> predictions, CarState checkstate) {

    cout << "Inside CostProvider:get_helper_data ...\n";
    TrajectoryData data = TrajectoryData ();

    vector<snapshot> t = trajectory;
    // actual state
    snapshot current_snapshot = t[0];
    snapshot first = t[1];
    snapshot last = t[t.size () - 1];

    double dt = trajectory.size () * PREDICTION_INTERVAL;
    // for lane change we see actual line after current state only
    data.current_lane = first.lane;
    data.proposed_lane = last.proposed_lane;
    data.avg_speed = (last.get_speed () * dt - current_snapshot.get_speed ()) / dt; // (v2*dt-v1*1)/dt

    // initialize a bunch of variables
    data.prop_closest_approach = MAX_DISTANCE;
    data.actual_closest_approach = MAX_DISTANCE;

    data.collides = collision ();
    data.collides.hasCollision = false;
    bool checkCollisions = current_snapshot.lane != data.proposed_lane;

    map<int, vector<prediction>> cars_in_proposed_lane = filter_predictions_by_lane (predictions, data.proposed_lane);
    map<int, vector<prediction>> cars_in_actual_lane = filter_predictions_by_lane (predictions, data.current_lane);

    if (verbose) {
        cout << "collides in lane: " << data.proposed_lane << " has cars: " << cars_in_proposed_lane.size () << endl;
        cout << "current lane: " << data.current_lane << " has cars: " << cars_in_actual_lane.size () << endl;
    }
    for (int i = 0; i < PLANNING_HORIZON; ++i) {
        snapshot snap = trajectory[i];

        for (auto pair : cars_in_actual_lane) {
            prediction state = pair.second[i];
            double dist = -state.get_distance (snap.x, snap.y, snap.s);
            if (dist >= 0 && dist < data.actual_closest_approach) {
                data.actual_closest_approach = dist;
            }
        }
    }

    for (int i = 0; i < PLANNING_HORIZON; ++i) {
        snapshot snap = trajectory[i];

        for (auto pair : cars_in_proposed_lane) {
            prediction state = pair.second[i];
            //double pred_car_s = car_s + i*0.15*ref_s;
            double dist = -state.get_distance (snap.x, snap.y, snap.s);
            if (checkCollisions) {
                bool vehicle_collides = check_collision (car_s, ref_s, snap, state, checkstate,
                                                         data.actual_closest_approach < MANOEUVRE);
                if (vehicle_collides) {
                    data.collides.hasCollision = true;
                    data.collides.step = i;
                } else if (car_s > state.s) {
                    dist = MAX_DISTANCE;
                }
            }
            if (dist >= 0 && dist < data.prop_closest_approach) {
                data.prop_closest_approach = dist;
                if (data.proposed_lane == data.current_lane) {
                    data.actual_closest_approach = data.prop_closest_approach;
                }
            }
        }
    }

    return data;
}


bool CostProvider::check_collision(double car_s, double ref_speed, snapshot snap, prediction s_now, CarState checkstate,
                                bool lack_of_space) {

    cout << "Inside CostProvider:check_collision ...\n";
    double s = snap.s;
    double v = snap.get_speed ();

    double collide_car_v = s_now.get_velocity ();
    double diff = s_now.get_distance (snap.x, snap.y, snap.s);
    double prediction_time = 4 / snap.dx;
    if (car_s > s_now.s) {// TODO
        double predicted_distance1v = diff + prediction_time * (v - collide_car_v);
        double predicted_distance2v = diff + 10 * PREDICTION_INTERVAL * (ref_speed - collide_car_v);
        if ((predicted_distance2v < MANOEUVRE || predicted_distance1v < MANOEUVRE || lack_of_space || diff < -1.0)) {
            if (verbose) {
                cout << "2nd clause: s " << s << " v " << v << " car_s: " << car_s << " col_v " << collide_car_v
                     << "obsticle: " << s_now.s << " diff " << diff << endl;
            }
            return true;
        }
    } else {
        double predicted_distance1v = -diff + 3 * PREDICTION_INTERVAL * (collide_car_v - v);
        if (predicted_distance1v < 0 || -diff < -MANOEUVRE) {
            if (verbose) {
                cout << "3rd clause: s " << s << " v " << v << " car_s: " << car_s << " col_v " << collide_car_v
                     << "obsticle: " << s_now.s << " diff " << diff << endl;
            }
            return true;
        }
    }

    return false;
}

map<int, vector<prediction>> CostProvider::filter_predictions_by_lane(
        map<int, vector<prediction>> predictions, int lane) {
    map<int, vector<prediction>> filtered = {};
    for (auto pair: predictions) {

        cout << "Inside CostProvider:filter_predictions_by_lane ...\n";
        // because of poor coord transformation reduce lane definition on 0.5m
        if (pair.second[0].lane == lane) {
            filtered[pair.first] = pair.second;
        }
    }
    return filtered;
}

