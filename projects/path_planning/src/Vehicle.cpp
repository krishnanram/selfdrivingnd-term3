#include "helper_functions.h"
#include "Vehicle.h"

#include <iostream>
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

#include "CostProvider.h"
#include "map.h"
#include "BehaviorPlanner.h"


using namespace helpers;

double prediction::LANE_WIDTH = 4.0;
double Vehicle::SAFE_DISTANCE = 20.0;

Vehicle::Vehicle(int id, double x, double y, double dx, double dy, double s, double d) {

    this->id = id;
    this->x = x;
    this->y = y;
    this->dx = dx;
    this->dy = dy;
    this->s = s;
    this->d = d;
    this->lane = (int) d / 4;
    this->ddx = 0;
    this->ddy = 0;
    double angle = atan2 (dy, dx);
    this->yaw = (abs (angle) < 0.1) ? 0 : angle;
    this->updates = 0;
}

Vehicle::Vehicle(int id) {
    this->id = id;
}

Vehicle::~Vehicle() {}

double Vehicle::get_velocity() {
    return sqrt (dx * dx + dy * dy);
}

bool Vehicle::shouldPredict() {

    cout << "Inside Vechile:shouldPredict ...\n";
    return true;
}

void Vehicle::display() {

    cout << "Inside Vechile:display ...\n";

    cout << "vehicle " << this->id << " info" << endl;
    cout << "s:    " << this->s;
    cout << " d: " << this->d;
    cout << " x:    " << this->x;
    cout << " y: " << this->y;
    cout << " yaw: " << this->yaw;
    cout << " vx:    " << this->dx;
    cout << " vy:    " << this->dy;
    cout << " ax:    " << this->ddx;
    cout << " ay:    " << this->ddy;
    cout << " line: " << this->lane << endl;
}

void Vehicle::update_params(double x, double y, double yaw, double s, double d, double speed, double diff) {

    cout << "Inside Vechile:update_params ...\n";

    this->x = x;
    this->y = y;
    this->yaw = deg2rad (yaw);
    update_accel (speed * cos (this->yaw), speed * sin (this->yaw), diff);
    this->s = s;
    this->d = d;
}

void Vehicle::update_accel(double vx, double vy, double diff) {

    cout << "Inside Vechile:update_accel ...\n";

    this->ddx = (vx - this->dx) / diff;
    if (this->ddx < 0.01) {
        this->ddx = 0;
    }
    this->ddy = (vy - this->dy) / diff;
    if (this->ddy < 0.01) {
        this->ddy = 0;

    }
    this->dx = vx;
    this->dy = vy;
}

void Vehicle::update_yaw(double x, double y, double vx, double vy, double s, double d, double diff) {

    cout << "Inside Vechile:update_yaw ...\n";

    double new_angle = atan2 (vy, vx);
    this->yaw = (abs (new_angle) < 0.1) ? 0 : new_angle;
    this->x = x;
    this->y = y;
    update_accel (vx, vy, diff);
    this->s = s;
    int new_lane = (int) d / 4;
    if (new_lane != this->lane) {
        if (++updates > 6) {
            this->lane = new_lane;
            updates = 0;
        }
    } else {
        updates = 0;
    }
    this->d = d;
}

void Vehicle::increment(double t) {

    cout << "Inside Vechile:increment ...\n";

    if (abs (this->ddy) < 0.001) {
        this->y += this->dy * t;
    } else {
        this->y += this->dy * t + this->ddy * t * t / 2;
        this->dy += this->ddy * t;
    }
    if (abs (this->ddx) < 0.001) {
        this->x += this->dx * t;
    } else {
        this->x += this->dx * t + this->ddx * t * t / 2;
        this->dx += this->ddx * t;
    }
    double new_angle = atan2 (dy, dx);
    this->yaw = (new_angle < 0.1) ? 0 : new_angle;
    Frenet frenet = Map::getFrenet (this->x, this->y, this->yaw);
    this->s = frenet.s;
    this->d = frenet.d;
}

prediction Vehicle::state_at(double t) {

    cout << "Inside Vechile:state_at ...\n";

    prediction pred;
    if (std::abs (this->ddy) < 0.001) {
        pred.y = this->y + this->dy * t;
        pred.vy = this->dy;
    } else {
        pred.y = this->y + this->dy * t + this->ddy * t * t / 2;
        pred.vy = this->dy + this->ddy * t;
    }
    if (std::abs (this->ddy) < 0.001) {
        pred.x = this->x + this->dx * t;
        pred.vx = this->dx;
    } else {
        pred.x = this->x + this->dx * t + this->ddx * t * t / 2;
        pred.vx = this->dx + this->ddx * t;
    }
    double new_angle = atan2 (pred.vy, pred.vx);
    double yaw = (new_angle < 0.1) ? 0 : new_angle;
    Frenet frenet = Map::getFrenet (pred.x, pred.y, yaw);
    pred.s = frenet.s;
    pred.d = frenet.d;
    pred.lane = this->lane;//TODO update if have future predictions
    return pred;
}

bool Vehicle::is_in_front_of(prediction pred, int checked_lane) {
    return (pred.lane == checked_lane) && pred.get_distance (x, y, s) >= 0;
}

bool Vehicle::is_behind_of(prediction pred, int lane) {
    double distance = -pred.get_distance (x, y, s);
    return (pred.lane == lane) && (distance >= 0 && distance < 2 * SAFE_DISTANCE);
}

bool Vehicle::is_close_to(prediction pred, int lane) {
    double distance = -pred.get_distance (x, y, s);
    return (pred.lane == lane) && distance >= 0 && (distance < SAFE_DISTANCE);
}

vector<prediction> Vehicle::generate_predictions(double interval, int horizon) {

    cout << "Inside Vechile:generate_predictions ...\n";

    vector<prediction> predictions;
    for (int i = 0; i < horizon; i++) {
        predictions.push_back (state_at (i * interval));
    }
    return predictions;
}


