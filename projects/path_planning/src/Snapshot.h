#ifndef SNAPSHOT_H
#define SNAPSHOT_H
#include <string>

using namespace std;

class Snapshot {
public:
    int lane;
    double s;
    double v;
    double a;
    string state;


    Snapshot(int lane, double s, double v, double a, string state);
    virtual ~Snapshot();


private :



};
#endif /* SNAPSHOT_H */
