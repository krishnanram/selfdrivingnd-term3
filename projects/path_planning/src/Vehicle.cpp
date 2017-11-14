#include "Vehicle.h"
#include <math.h>
#include <iostream>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>
#include "config.h"
#include "Snapshot.h"
#include "CostProvider.h"
#include "Utility.h"

//  --------------------------- Public  ------------------------------------------------------//

Vehicle::Vehicle(int lane, double s, double v, double a, bool my_self_driving_car) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = "CS";

  if (my_self_driving_car) {

    // configure speed limit, num_lanes, max_acceleration,
    // max_decceleration, and my_self_driving_car for my_self_driving_car vehicle

    this->target_speed = MY_SELF_DRIVING_CAR_MAX_VELOCITY;
    this->lanes_available = NUM_LANES;
    this->max_acceleration = MY_SELF_DRIVING_CAR_MAX_ACCEL;
    this->max_decceleration = MY_SELF_DRIVING_CAR_MAX_DECEL;
    this->my_self_driving_car = true;

  } else {
    this->max_acceleration = 0;
    this->max_decceleration = 0;
  }

  this->cost = CostProvider();
}

Vehicle::~Vehicle() {}

void Vehicle::update_state(map<int, vector <vector<double>>> predictions) {
  /*
    Updates the "state" of the my_self_driving_car vehicle by assigning one of the
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
  this->state = this->get_next_state(predictions);
}

string Vehicle::get_next_state(map<int, vector <vector<double>>> predictions) {

  vector<string> states = {"KL", "PLCR", "LCR", "PLCL", "LCL"};

  // remove states that are impossible to get to from the current state
  string state = this->state;

  if (this->lane == 0) {  // left-hand lane
    states.erase(states.begin() + 4);
    states.erase(states.begin() + 3);

    // if in KL state then only allow
    // transition to PLCR, not LCR
    if (state.compare("KL") == 0) states.erase(states.begin() + 2);

  } else if (this->lane == 1) {  // middle lane
    // if in KL state, only allow transition
    //  to PLCR or PLCL, not LCR or LCL
    if (state.compare("KL") == 0) {
      states.erase(states.begin() + 4);
      states.erase(states.begin() + 2);
    }
  } else if (this->lane == this->lanes_available - 1) {  // right-hand lane
    // if in KL state, only allow transition
    // to PLCR, not LCR
    if (state.compare("KL") == 0) states.erase(states.begin() + 4);

    states.erase(states.begin() + 2);
    states.erase(states.begin() + 1);
  }

  vector<string> new_states;
  vector<double> new_costs;

  for (int i = 0; i < states.size(); i++) {
    string state = states[i];

    // two trajectories - the current trajectory, and the proposed state trajectory
    vector<Snapshot> trajectories = this->trajectories_for_state(state, predictions, TRAJECTORIES_HZ);

    double cost = this->cost.calculate_cost(*this, trajectories, predictions);

    new_states.insert(new_states.end(), state);
    new_costs.insert(new_costs.end(), cost);
  }

  string best = this->min_cost_state(new_states, new_costs);

  return best;
}

string Vehicle::min_cost_state(vector<string> states, vector<double> costs) {
  string best_state = "";
  double best_cost = 999999999.0;

  if (DEBUG) cout << endl << "State Costs: ";

  for (int i = 0; i < states.size(); i++) {
    if (DEBUG) {
      if (i > 0) cout << ", ";
      cout << states[i] << ": " << costs[i];
    }

    if (costs[i] < best_cost) {
      best_cost = costs[i];
      best_state = states[i];
    }
  }

  if (DEBUG) cout << endl << endl;

  return best_state;
}

vector<Snapshot> Vehicle::trajectories_for_state(string state, map<int, vector<vector<double>>> predictions, int horizon) {

  // remember the current state of my_self_driving_car
  Snapshot current = Snapshot(this->lane, this->s, this->v, this->a, this->state);

  // build a list of trajectories
  vector<Snapshot> trajectories;

  // save the current state for the initial trajectory in the list
  trajectories.insert(trajectories.end(), current);

  // ...pretend to be in the new proposed state
  this->state = state;

  // perform the state transition for the proposed state
  this->realize_state(predictions);

  for (int i = 0; i < horizon; i++) {
    // update the velocity and acceleration of my_self_driving_car out to the horizon
    this->increment(1.0, true);
  }

  // save the trajectory results of the proposed state
  trajectories.insert(trajectories.end(), Snapshot(this->lane, this->s, this->v, this->a, this->state));

  // need to remove first prediction for each vehicle
  map<int, vector <vector<double>>>::iterator pr = predictions.begin();
  while (pr != predictions.end()) {
    int prediction_id = pr->first;

    vector<vector<double>> prediction = pr->second;

    prediction.erase(prediction.begin());

    predictions[prediction_id] = prediction;

    pr++;
  }

  // restore the my_self_driving_car state from the snapshot
  this->lane = current.lane;
  this->s = current.s;
  this->v = current.v;
  this->a = current.a;
  this->state = current.state;

  return trajectories;
}


// NOTE: we skip the calculation of the s-value when we
// are "fer-realz" incrementing; otherwise calculated
// s-value gets out of sync quickly with the actual
// s-value from the simulator

void Vehicle::increment(double dt, bool overide) {
  double ddt = dt * SECS_PER_TICK;

  // NOTE: if we are updating the my_self_driving_car , don't calculate the
  // s-value, keep the currently set value and don't vary it
  if (!this->my_self_driving_car || overide) this->s += this->v * ddt;

  this->v += this->a * ddt;

  if (this->my_self_driving_car) {
    this->v = max(0.0, this->v);  // don't allow my_self_driving_car velocity to go negative
    this->v = min(MY_SELF_DRIVING_CAR_MAX_VELOCITY, this->v);  // ...or go over the speed limit either
  }
}

vector<double> Vehicle::state_at(double t) {
  /*
    Predicts state of vehicle in t*0.02 seconds (assuming constant acceleration)
  */
  double dt = t * SECS_PER_TICK;
  double s = this->s + this->v * dt + this->a * dt * dt / 2;
  double v = this->v + this->a * dt;
  if (this->my_self_driving_car) v = max(0.0, v);  // don't allow the my_self_driving_car's velocity to go negative
  return {static_cast<double>(this->lane), s, v, this->a};
}




void Vehicle::realize_state(map<int, vector<vector<double>>> predictions) {
  /*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
  */
  string state = this->state;

  if (state.compare("CS") == 0) {
    realize_constant_speed();
  } else if (state.compare("KL") == 0) {
    realize_keep_lane(predictions);
  } else if (state.compare("LCL") == 0) {
    realize_lane_change(predictions, "L");
  } else if (state.compare("LCR") == 0) {
    realize_lane_change(predictions, "R");
  } else if (state.compare("PLCL") == 0) {
    realize_prep_lane_change(predictions, "L");
  } else if (state.compare("PLCR") == 0) {
    realize_prep_lane_change(predictions, "R");
  }
}

void Vehicle::realize_constant_speed() {
  // set acceleration to zero for keeping a constant speed
  this->a = 0;
}

void Vehicle::realize_keep_lane(map<int, vector<vector<double>>> predictions) {
  // continue to get the acceleration for the current lane
  this->a = this->_max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int, vector<vector<double>>> predictions, string direction) {
  // doing a lane change, so based on which direction (left/right)
  // for the new lane being changed to, set a delta offset to
  // represent the new lane
  double delta = 1;

  if (direction.compare("L") == 0) {
    delta = -1;
  }

  // change to the new lane
  this->lane += delta;

  // prevent straying beyond minimum or maximum lane positions
  this->lane = minmaxCarLaneNumber(this->lane);

  // get acceleration for new lane
  this->a = this->_max_accel_for_lane(predictions, this->lane, this->s);
}

double Vehicle::_max_accel_for_lane(map<int, vector<vector<double>>> predictions, int lane, double ss) {
  // at each time step we need to gradually approach the target
  // speed by ramping up the acceleration, to its maximum limit
  double delta_v_til_target = this->target_speed - this->v;
  double max_acc = min(this->max_acceleration, delta_v_til_target);

  // build a list of all vehicles directly in front of the my_self_driving_car
  map<int, vector<vector<double>>>::iterator it = predictions.begin();
  vector<vector<vector<double>>> in_front;

  while (it != predictions.end()) {
    int vv_id = it->first;

    vector<vector<double>> vv = it->second;

    if ((static_cast<int>(vv[0][0]) == lane) && (vv[0][1] > ss)) {
      in_front.push_back(vv);
    }

    it++;
  }

  // if there are any cars in front of the my_self_driving_car , find
  // the first car in the lane (the one directly ahead)
  this->in_front = in_front.size();

  if (this->in_front > 0) {
    double min_s = LEADING_HZ;  // how far to look ahead

    vector<vector<double>> leading;

    for (int i = 0; i < in_front.size(); i++) {
      if ((in_front[i][0][1] - ss) < min_s) {
        min_s = (in_front[i][0][1] - ss);
        leading = in_front[i];
      }
    }

    if (leading.size() > 1) {
      // now that we have found the car directly in front of the
      // my_self_driving_car , find where it will be next...
      double next_pos = leading[1][1];
      // then find where the my_self_driving_car  will be next based on its speed
      double my_next = ss + this->v;
      // find out how far apart they will be from each other at that time
      double separation_next = next_pos - my_next;
      // subtract a bit of buffer room for comfort, and that's the
      // available room the my_self_driving_car  has to maneuver in
      double available_room = separation_next - PREFERRED_BUFFER;

      // keep going at current speed if there is available room,
      // otherwise reduce speed...
      max_acc = min(max_acc, available_room);
      // but don't let the acceleration fall below the minimum
      max_acc = max(max_acc, -this->max_decceleration);
    }
  }

  return max_acc;
}

void Vehicle::realize_prep_lane_change(map<int, vector<vector<double>>> predictions, string direction) {
  // prepping for a lane change, so based on which lane is
  // going to be changed to, set a delta offset to represent
  // the new lane
  int delta = 1;

  if (direction.compare("L") == 0) {
    delta = -1;
  }

  // set the lane to check based on the delta offset value
  int lane = this->lane + delta;

  // build a list of all vehicles next to or behind the my_self_driving_car

  map<int, vector<vector<double>>>::iterator it = predictions.begin();

  vector<vector<vector<double>>> at_behind;

  while (it != predictions.end()) {
    int v_id = it->first;

    vector<vector<double>> v = it->second;

    if ((static_cast<int>(v[0][0]) == lane) && (v[0][1] <= this->s)) {
      at_behind.push_back(v);
    }

    it++;
  }

  // if there are any cars found behind the my_self_driving_car ...
  this->at_behind = at_behind.size();

  if (this->at_behind > 0) {
    double max_s = -FOLLOW_HZ;  // how far to look behind

    // find the closest vehicle behind the my_self_driving_car
    vector<vector<double>> nearest_behind;

    for (int i = 0; i < at_behind.size(); i++) {
      if ((at_behind[i][0][1]) > max_s) {
        max_s = at_behind[i][0][1];
        nearest_behind = at_behind[i];
      }
    }

    // get the diffence in the velocity of the trailing vehicle
    double target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    // the my_self_driving_car  needs to change from its current velocity
    double delta_v = this->v - target_vel;
    // ...and the my_self_driving_car  needs to get behind the trailing vehicle
    double delta_s = this->s - nearest_behind[0][1];

    // if the trailing car is changing its velocity...
    if (delta_v != 0) {
      // ...then we need to compute the acceleration value needed
      // to match the change in order to get behind it
      double time = -2.0 * delta_s / delta_v;
      double aa;

      if (time == 0) {
        aa = this->a;
      } else {
        aa = delta_v / time;
      }

      // keep acceleration within boundaries of min/max acceleration
      if (aa > this->max_acceleration) {
        aa = this->max_acceleration;
      }

      if (aa < -this->max_decceleration) {
        aa = -this->max_decceleration;
      }

      this->a = aa;
    } else {
      // if the trailing vehicle isn't changing its speed, then the
      // my_self_driving_car  just needs to slow down
      double my_min_acc = max(-this->max_decceleration, -delta_s);

      this->a = my_min_acc;
    }
  } else {
    // otherwise with no cars found behind my_self_driving_car, continue at regular speed
    this->a = this->_max_accel_for_lane(predictions, this->lane, this->s);
  }
}

vector<vector<double>> Vehicle::generate_predictions(int horizon) {
  vector<vector<double>> predictions;

  for (int i = 0; i < horizon; i++) {
    vector<double> check = this->state_at(static_cast<double>(i));
    vector<double> lane_s = {check[0], check[1]};
    predictions.push_back(lane_s);
  }

  return predictions;
}

//  --------------------------- Private  ------------------------------------------------------//