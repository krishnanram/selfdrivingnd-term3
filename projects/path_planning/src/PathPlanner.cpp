#include <chrono>

#include "json.hpp"
#include "Vehicle.h"
#include "map.h"
#include "BehaviorPlanner.h"
#include "Trajectory.h"

#include "PathPlanner.h"


//using namespace std;
//using json = nlohmann::json;
//using namespace std::chrono;

PathPlanner::PathPlanner() {
    ms = duration_cast<milliseconds> (
            system_clock::now ().time_since_epoch ()
    );

}

double PathPlanner::get_time_step() {

    cout << "Inside PathPlanner:get_time_step ...\n";

    milliseconds new_time = duration_cast<milliseconds> (system_clock::now ().time_since_epoch ());
    diff = (double) (new_time - ms).count () / 1000;
    ms = new_time;
    return diff;
}

PathPlanner::~PathPlanner() {}


// [car's unique ID, car's x position in map coordinates, car's y position in map coordinates,
// car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates,
// car's d position in frenet coordinates.
void PathPlanner::update_vehicle_state(json sensor_fusion) {

    cout << "Inside PathPlanner:update_vehicle_state ...\n";

    double diff = get_time_step ();

    predictions.clear ();
    for (auto data : sensor_fusion) {
        // [id, x, y, dx, dy, s, d]
        Vehicle *vehicle = NULL;
        if (((double) data[5] <= Map::MAX_S) && ((double) data[6] >= 0)) {// check if car is visible
            if (vehicles.find (data[0]) == vehicles.end ()) {

                vehicle = new Vehicle (data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
                vehicles[data[0]] = vehicle;
            } else {
                vehicle = vehicles[data[0]];
                (*vehicle).update_yaw (data[1], data[2], data[3], data[4], data[5], data[6], diff);
                if ((*vehicle).shouldPredict ()) {
                    vector<prediction> car_preds = (*vehicle).generate_predictions (
                            behaviorPlanner.PREDICTION_INTERVAL);
                    predictions[(*vehicle).id] = car_preds;
                }
            }
        } else {
            auto it = vehicles.find (data[0]);
            if (it != vehicles.end ()) {
                cout << " remove vehicle: " << data[0] << endl;
                delete (*it).second;
                vehicles.erase ((int) data[0]);
            }
        }
    }
}

void PathPlanner::update_my_self_driving_car_car_state(double car_s, double x, double y, double yaw, double s, double d, double speed) {

    cout << "Inside PathPlanner:update_my_self_driving_car_car_state ...\n";

    original_yaw = yaw;
    behaviorPlanner.car_s = car_s;
    behaviorPlanner.my_self_driving_car_car.update_params (x, y, yaw, s, d, speed, diff);
}

void PathPlanner::generate_trajectory(vector<double> previous_path_x, vector<double> previous_path_y) {

    cout << "Inside PathPlanner:generate_trajectory ...\n";

    behaviorPlanner.update_state (predictions);
    behaviorPlanner.realize_state (predictions);
    trajectory.set_previous_path (previous_path_x, previous_path_y);
    trajectory.generate_trajectory (behaviorPlanner.car_s, my_self_driving_car_car.x, my_self_driving_car_car.y, original_yaw, my_self_driving_car_car.lane,
                                    behaviorPlanner.get_expected_velocity ());
}

vector<double> PathPlanner::get_x_values() {

    cout << "Inside PathPlanner:get_x_values ...\n";
    return trajectory.next_x_vals;
}

vector<double> PathPlanner::get_y_values() {

    cout << "Inside PathPlanner:get_y_values ...\n";
    return trajectory.next_y_vals;
}
