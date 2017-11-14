#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <cmath>
#include <vector>
#include "Road.h"
#include "Vehicle.h"

using namespace std;

class BehaviorPlanner {

public:

    BehaviorPlanner();
    virtual ~BehaviorPlanner();

    void advance(vector<vector<double>> sensor_fusion, double my_self_driving_car_s = 0);
    Vehicle get_my_self_driving_car();


private :

    Road road;

};
#endif /* BEHAVIOR_H */
