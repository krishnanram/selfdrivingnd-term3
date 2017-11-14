#include "CostProvider.h"
#include <math.h>
#include <iostream>
#include <map>
#include <string>
#include <iterator>
#include "config.h"
#include "Vehicle.h"
#include "Snapshot.h"


//  --------------------------- Public  ------------------------------------------------------//

CostProvider::CostProvider() {}
CostProvider::~CostProvider() {}

double CostProvider::calculate_cost(const Vehicle &vehicle, vector<Snapshot> trajectories, map<int, vector<vector<double>>> predictions) {

  TrajectoryData trajectory_data = this->get_trajectory_data(vehicle, trajectories, predictions);

  double cost = 0;

  cost += this->inefficiency_cost(vehicle, trajectories, predictions, trajectory_data);
  cost += this->collision_cost(vehicle, trajectories, predictions, trajectory_data);
  cost += this->buffer_cost(vehicle, trajectories, predictions, trajectory_data);

  return cost;
}

TrajectoryData CostProvider::get_trajectory_data(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<double>>> predictions) {
  TrajectoryData trajectory_data;

  vector<Snapshot> t = trajectories;

  Snapshot current_snapshot = t[0];
  Snapshot first = t[1];
  Snapshot last = t[t.size() - 1];

  double dt = trajectories.size();

  trajectory_data.proposed_lane = first.lane;

  trajectory_data.avg_speed = (last.s - current_snapshot.s) / dt;

  // initialize a bunch of variables
  vector<double> accels;

  double closest_approach = 9999999.0;
  bool collides = false;

  map<int, vector<vector<double>>> filtered = this->filter_predictions_by_lane(predictions, trajectory_data.proposed_lane);

  // get trajectory for future state of my_self_driving_car
  Snapshot snapshot = trajectories[1];

  accels.insert(accels.end(), snapshot.a);

  map<int, vector<vector<double>>>::iterator it = filtered.begin();

  while (it != filtered.end()) {
    int v_id = it->first;

    vector<vector<double>> v = it->second;
    vector<double> curr_state = v[1];
    vector<double> prev_state = v[0];

    bool vehicle_collides = this->check_collision(snapshot, prev_state[1], curr_state[1]);
    if (vehicle_collides) {
      trajectory_data.collides = true;
      trajectory_data.collides_at = curr_state[1];
    }

    int dist = abs(curr_state[1] - snapshot.s);
    if (dist < trajectory_data.closest_approach) trajectory_data.closest_approach = dist;

    it++;
  }

  // find max acceleration value
  int num_accels = accels.size();
  trajectory_data.max_acceleration = 0;
  for (int i = 0; i < num_accels; i++) {
    if (accels[i] > trajectory_data.max_acceleration) trajectory_data.max_acceleration = accels[i];
  }

  return trajectory_data;
}

map<int, vector<vector<double>>> CostProvider::filter_predictions_by_lane(map<int, vector <vector<double>>> predictions, int lane) {
  map<int, vector<vector<double>>> filtered;

  map<int, vector<vector<double>>>::iterator it = predictions.begin();

  while (it != predictions.end()) {
    int v_id = it->first;
    vector<vector<double>> predicted_traj = it->second;

    if (static_cast<int>(predicted_traj[0][0]) == lane && v_id != -1) {
      filtered[v_id] = predicted_traj;
    }

    it++;
  }

  return filtered;
}

bool CostProvider::check_collision(Snapshot snapshot, double s_prev, double s_curr) {
  double v_target = s_curr - s_prev;

  if (s_prev < snapshot.s) return (s_curr >= snapshot.s);
  if (s_prev > snapshot.s) return (s_curr <= snapshot.s);
  if (s_prev == snapshot.s) return (v_target <= snapshot.v);

  return true;
}

double CostProvider::inefficiency_cost(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<double>>> predictions, TrajectoryData data) {
  double speed = data.avg_speed;
  double target_speed = vehicle.target_speed;
  double diff = target_speed - speed;
  double pct = diff / target_speed;
  double multiplier = pct * pct;
  return multiplier * EFFICIENCY;
}

double CostProvider::collision_cost(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<double>>> predictions, TrajectoryData data) {
  if (data.collides) {
    double time_til_collision = data.collides_at;
    double exponent = time_til_collision * time_til_collision;
    double multiplier = exp(-exponent);
    return multiplier * COLLISION;
  }

  return 0.0;
}

double CostProvider::buffer_cost(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<double>>> predictions, TrajectoryData data) {
  double closest = data.closest_approach;
  if (closest == 0) return 10 * DANGER;

  double timesteps_away = closest / data.avg_speed;
  if (timesteps_away > PREFERRED_BUFFER) return 0;

  double multiplier = 1 - pow((timesteps_away / PREFERRED_BUFFER), 2);

  return multiplier * DANGER;
}

//  --------------------------- Private  ------------------------------------------------------//