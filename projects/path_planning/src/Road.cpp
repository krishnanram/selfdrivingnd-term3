#include "Road.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include <utility>
#include "config.h"
#include "Vehicle.h"
#include "Utility.h"


//  --------------------------- Public  ------------------------------------------------------//

Road::Road() {}
Road::~Road() {}

Vehicle Road::get_my_self_driving_car() {
    return this->vehicles.find(this->my_self_driving_car_key)->second;
}

void Road::populate_traffic(vector<vector<double>> sensor_fusion) {

    // clear vehicle list of every vehicle except my_self_driving_car

    map<int, Vehicle>::iterator i = this->vehicles.begin();
    while (i != this->vehicles.end()) {
        int this_oah_id = i->first;
        if (this_oah_id == this->my_self_driving_car_key) {
            i++;  // skip my_self_driving_car
        } else {
            i = this->vehicles.erase(i);  // delete npc vehicle
        }
    }

    // repopulate vehicle list with sensor data
    for (int i = 0; i < sensor_fusion.size(); i++) {
        int oah_id = static_cast<int>(sensor_fusion[i][0]);  // unique id for car

        double oah_x = sensor_fusion[i][1];   // x-position global map coords
        double oah_y = sensor_fusion[i][2];   // y-position global map coords
        double oah_vx = sensor_fusion[i][3];  // x-component of car's velocity
        double oah_vy = sensor_fusion[i][4];  // y-component of car's velocity
        double oah_s = sensor_fusion[i][5];   // how far down the road is the car?
        double oah_d = sensor_fusion[i][6];   // what lane is the car in?

        int oah_lane = getLaneNumber(oah_d);

        // compute the magnitude of the velocity (distance formula)
        double oah_speed = sqrt(oah_vx * oah_vx + oah_vy * oah_vy);

        Vehicle vehicle = Vehicle(oah_lane, oah_s, oah_speed);
        vehicle.state = "KL";
        this->vehicles.insert(std::pair<int, Vehicle>(oah_id, vehicle));
    }
}

void Road::advance(double my_self_driving_car_s) {
    map<int, vector<vector<double>>> predictions;

    map<int, Vehicle>::iterator it = this->vehicles.begin();
    while (it != this->vehicles.end()) {
        int v_id = it->first;

        Vehicle vv = it->second;

        // if the lane is < 0 then vehicle is not on track, so skip
        if (vv.lane > -1) {
            predictions[v_id] = vv.generate_predictions(PREDICTIONS_HZ);
        }

        it++;
    }

    it = this->vehicles.begin();
    while (it != this->vehicles.end()) {
        int v_id = it->first;

        bool is_my_self_driving_car = (v_id == my_self_driving_car_key);

        ticks++;
        if (ticks > MAX_TICKS) {
            ticks = 0;
            if (is_my_self_driving_car) {
                it->second.s = my_self_driving_car_s;  // sync up my_self_driving_car s-value to simulator s-value
                it->second.update_state(predictions);
                it->second.realize_state(predictions);
            }
        }

        it->second.increment();

        it++;
    }
}

void Road::add_my_self_driving_car(int lane_num, double s) {
    Vehicle my_self_driving_car = Vehicle(lane_num, s, 0, 0, true);

    this->vehicles.insert(std::pair<int, Vehicle>(this->my_self_driving_car_key, my_self_driving_car));
}


//  --------------------------- Private  ------------------------------------------------------//