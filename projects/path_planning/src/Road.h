#ifndef ROAD_H
#define ROAD_H

#include <math.h>
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "config.h"
#include "Vehicle.h"

using namespace std;

class Road {
public:


    Road();
    virtual ~Road();

    Vehicle get_my_self_driving_car();
    void populate_traffic(vector<vector<double>> sensor_fusion);
    void advance(double my_self_driving_car_s = 0);
    void add_my_self_driving_car(int lane_num, double s);


private :

    int ticks = MAX_TICKS;
    map<int, Vehicle> vehicles;
    int my_self_driving_car_key = -1;  // identifier for my_self_driving_car in vehicles map


};
#endif /* ROAD_H */
