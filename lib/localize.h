#ifndef LOCALIZE_H
#define LOCALIZE_H

#include <math.h>

struct Pose {
  float x;
  float y;
  float heading;
};

static Pose _loc_pose;

static void loc_reset() {
  _loc_pose.x = 0;
  _loc_pose.y = 0;
  _loc_pose.heading = 0;
}

static Pose loc_get() {
  return _loc_pose;
}

static void loc_update_forward(float cm) {
  float rad = _loc_pose.heading * M_PI / 180.0;
  _loc_pose.x += cm * sin(rad);
  _loc_pose.y += cm * cos(rad);
}

static void loc_update_turn(float degrees) {
  _loc_pose.heading = fmod(_loc_pose.heading + degrees, 360.0);
  if (_loc_pose.heading < 0) {
    _loc_pose.heading += 360.0;
  }
}

static void loc_correct(float x, float y) {
  _loc_pose.x = x;
  _loc_pose.y = y;
}

#endif
