#include "BehaviorPlanner.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <string>
#include "config.h"
#include "Road.h"

//  --------------------------- Public  ------------------------------------------------------//
BehaviorPlanner::BehaviorPlanner() {
  this->road = Road();

  this->road.add_my_self_driving_car(MY_SELF_DRIVING_CAR_START_LANE, 0);
}

BehaviorPlanner::~BehaviorPlanner() {}

void BehaviorPlanner::advance(vector<vector<double>> sensor_fusion, double my_self_driving_car_s) {
  this->road.populate_traffic(sensor_fusion);

    this->road.advance(my_self_driving_car_s);
}

Vehicle BehaviorPlanner::get_my_self_driving_car() {
  return this->road.get_my_self_driving_car();
}


//  --------------------------- Private  ------------------------------------------------------//