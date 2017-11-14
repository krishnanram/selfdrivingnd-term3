#include "Snapshot.h"
#include <string>


//  --------------------------- Public  ------------------------------------------------------//

Snapshot::Snapshot(int lane, double s, double v, double a, string state) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;
}

Snapshot::~Snapshot() {}

//  --------------------------- Private  ------------------------------------------------------//