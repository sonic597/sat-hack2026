#ifndef LOCALIZE_H
#define LOCALIZE_H

// Dead-reckoning localization for maze navigation (all stages).
// Tracks robot pose (x, y, heading) by integrating forward motion and
// turn commands. No sensor feedback — drifts over time; use particle_filter.h
// to correct.
//
// Memory: 12 bytes (one Pose struct).
// Timing: all functions O(1), < 1us on 16MHz AVR.
//
// Setup (in setup()):
//
//   #include "localize.h"
//
//   loc_reset();  // zero pose at starting position
//
// Drive loop (called after each motion command):
//
//   loc_update_forward(cm);   // after driving straight
//   loc_update_turn(degrees); // after turning (positive = clockwise)
//
//   Pose p = loc_get();       // read current estimate
//
// Correction (optional, called with particle filter output):
//
//   loc_correct(est.x, est.y);  // snaps x/y; heading is unchanged
//
// Notes:
//   - Coordinate frame: +X = right, +Y = forward, heading 0 = forward.
//   - Heading is kept in [0, 360) degrees.
//   - loc_correct only updates position, not heading. Feed heading
//     corrections via loc_update_turn if needed.

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
