#ifndef HELPER_H
#define HELPER_H

#include <cmath>
#include <vector>

using namespace std;


const int LANE_WIDTH = 4;  // width in Meters
//const int LANE_CENTR = LANE_WIDTH  / 2;

constexpr double pi() { return M_PI; }
constexpr double tau() { return M_PI * 2; }
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

int getLaneNumber(double d);

int getLaneFrenet(int lane, int offset);

int minmaxCarLaneNumber(int lane);

#endif /* HELPER_H */
