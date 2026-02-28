#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

// Particle filter localization for maze navigation (stages 2 & 3).
// Fuses dead-reckoning motion (from localize.h) with ultrasonic sensor
// readings against a 2-bit packed occupancy grid to correct pose drift.
//
// Memory: ~714 bytes steady-state, ~954 bytes peak (during resample).
// Timing: pf_update ~35-55ms on 16MHz AVR, pf_predict ~1ms.
//
// Setup (in setup()):
//
//   #include "localize.h"
//   #include "particle_filter.h"
//
//   // After mapping is initialized:
//   pf_set_map(map_get_grid(), MAP_W, MAP_H, CELL_CM);
//
//   // Sensor offset from robot center in body frame (cm).
//   // Body +X = right, +Y = forward.
//   pf_set_sensor_offset(0.0, 5.0);  // sensor 5cm ahead of center
//
//   // Seed particles at starting position (spread in cm and degrees).
//   pf_init(start_x, start_y, start_heading, 5.0, 5.0);
//
// Drive loop (called after each motion command):
//
//   // 1. Tell both dead-reckoning and particle filter about motion.
//   loc_update_forward(cm);
//   pf_predict_forward(cm, 2.0);   // 2cm noise std dev
//
//   loc_update_turn(deg);
//   pf_predict_turn(deg, 3.0);     // 3deg noise std dev
//
//   // 2. Take a sensor reading and update the filter.
//   float dist = read_ultrasonic_cm();
//   pf_update(dist);  // weights + resamples internally
//
//   // 3. Get corrected pose estimate.
//   Pose est = pf_estimate();
//
//   // 4. Optionally feed back into dead-reckoning to prevent drift.
//   loc_correct(est.x, est.y);
//
// Notes:
//   - pf_update skips readings <= 2cm or >= 299cm automatically.
//   - Tolerant of occasional outlier readings (e.g. through wall gaps)
//     thanks to a uniform mixture likelihood floor.
//   - If all particles lose track, it reinitializes from loc_get().
//   - The grid must use 2-bit packing: byte[idx/4], bits (idx%4)*2,
//     where 0=FREE, 1=UNKNOWN, 2=WALL, 3=DYNAMIC.

#include "localize.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

#define PF_N             15
#define PF_MAX_RANGE     300.0f
#define PF_SENSOR_SIGMA  20.0f

// ---------- internal types ----------

struct Particle {
  float x;
  float y;
  float heading; // degrees, [0, 360)
  float weight;
};

// ---------- internal state ----------

static Particle _pf_particles[PF_N];

static uint8_t* _pf_grid;
static int      _pf_grid_w;
static int      _pf_grid_h;
static int      _pf_cell_cm;

static float _pf_sensor_ox;
static float _pf_sensor_oy;

// ---------- internal helpers ----------

// Approximate standard-normal sample via sum of 3 uniforms.
// Mean ~0, stddev ~0.577 * sqrt(3) ≈ 1.0
static float _pf_approx_gaussian() {
  float sum = 0;
  for (int i = 0; i < 3; i++) {
    sum += random(-1000, 1001) / 1000.0f;
  }
  return sum; // stddev ≈ 1
}

static float _pf_wrap_heading(float h) {
  h = fmod(h, 360.0f);
  if (h < 0) h += 360.0f;
  return h;
}

// Read 2-bit cell from packed grid. Out-of-bounds returns 2 (WALL).
static uint8_t _pf_grid_get(int gx, int gy) {
  if (gx < 0 || gx >= _pf_grid_w || gy < 0 || gy >= _pf_grid_h)
    return 2;
  int idx = gy * _pf_grid_w + gx;
  uint8_t byte_val = _pf_grid[idx / 4];
  int shift = (idx % 4) * 2;
  return (byte_val >> shift) & 0x03;
}

// Ray-cast from (sx, sy) along heading_rad until hitting WALL(2) or DYNAMIC(3).
// UNKNOWN(1) is treated as passable. Returns distance in cm.
static float _pf_ray_cast(float sx, float sy, float heading_rad) {
  float step = _pf_cell_cm * 0.5f;
  float dx = sin(heading_rad) * step;
  float dy = cos(heading_rad) * step;
  float cx = sx;
  float cy = sy;
  float dist = 0;

  while (dist < PF_MAX_RANGE) {
    cx += dx;
    cy += dy;
    dist += step;

    int gx = (int)(cx / _pf_cell_cm);
    int gy = (int)(cy / _pf_cell_cm);
    uint8_t cell = _pf_grid_get(gx, gy);
    if (cell == 2 || cell == 3) { // WALL or DYNAMIC
      return dist;
    }
  }
  return PF_MAX_RANGE;
}

// ---------- public API ----------

static void pf_init(float x, float y, float heading,
                     float spread_cm, float spread_deg) {
  for (int i = 0; i < PF_N; i++) {
    _pf_particles[i].x       = x + _pf_approx_gaussian() * spread_cm;
    _pf_particles[i].y       = y + _pf_approx_gaussian() * spread_cm;
    _pf_particles[i].heading = _pf_wrap_heading(
        heading + _pf_approx_gaussian() * spread_deg);
    _pf_particles[i].weight  = 1.0f / PF_N;
  }
}

static void pf_set_map(uint8_t* grid, int width, int height, int cell_cm) {
  _pf_grid    = grid;
  _pf_grid_w  = width;
  _pf_grid_h  = height;
  _pf_cell_cm = cell_cm;
}

static void pf_set_sensor_offset(float ox, float oy) {
  _pf_sensor_ox = ox;
  _pf_sensor_oy = oy;
}

static void pf_predict_forward(float cm, float noise_cm) {
  for (int i = 0; i < PF_N; i++) {
    float d = cm + _pf_approx_gaussian() * noise_cm;
    float rad = _pf_particles[i].heading * (float)M_PI / 180.0f;
    _pf_particles[i].x += d * sin(rad);
    _pf_particles[i].y += d * cos(rad);
  }
}

static void pf_predict_turn(float degrees, float noise_deg) {
  for (int i = 0; i < PF_N; i++) {
    float d = degrees + _pf_approx_gaussian() * noise_deg;
    _pf_particles[i].heading = _pf_wrap_heading(
        _pf_particles[i].heading + d);
  }
}

static void pf_update(float sensor_dist_cm) {
  // Skip invalid / extreme readings
  if (sensor_dist_cm <= 2.0f || sensor_dist_cm >= PF_MAX_RANGE - 1.0f)
    return;

  float total_weight = 0;

  for (int i = 0; i < PF_N; i++) {
    float h_rad = _pf_particles[i].heading * (float)M_PI / 180.0f;

    // Sensor world position (body +X = right, +Y = forward)
    float sx = _pf_particles[i].x
               + _pf_sensor_ox * cos(h_rad)
               + _pf_sensor_oy * sin(h_rad);
    float sy = _pf_particles[i].y
               - _pf_sensor_ox * sin(h_rad)
               + _pf_sensor_oy * cos(h_rad);

    float expected = _pf_ray_cast(sx, sy, h_rad);
    float diff = sensor_dist_cm - expected;
    float gauss = exp(-0.5f * diff * diff
                      / (PF_SENSOR_SIGMA * PF_SENSOR_SIGMA));
    // Uniform mixture floor: 10% chance the reading is a random outlier.
    // Prevents a single bad reading (e.g. through wall gap) from
    // zeroing all particle weights.
    float likelihood = 0.9f * gauss + 0.1f / PF_MAX_RANGE;

    _pf_particles[i].weight *= likelihood;
    total_weight += _pf_particles[i].weight;
  }

  // Particle deprivation guard
  if (total_weight < 1e-10f) {
    Pose p = loc_get();
    pf_init(p.x, p.y, p.heading, 10.0f, 15.0f);
    return;
  }

  // Normalize
  for (int i = 0; i < PF_N; i++) {
    _pf_particles[i].weight /= total_weight;
  }

  // Low-variance resampling
  Particle temp[PF_N];
  float r = (random(0, 1000) / 1000.0f) / PF_N;
  float c = _pf_particles[0].weight;
  int j = 0;

  for (int i = 0; i < PF_N; i++) {
    float u = r + (float)i / PF_N;
    while (c < u && j < PF_N - 1) {
      j++;
      c += _pf_particles[j].weight;
    }
    temp[i] = _pf_particles[j];
    temp[i].weight = 1.0f / PF_N;
  }

  memcpy(_pf_particles, temp, sizeof(temp));

  // Roughening: small jitter to maintain diversity
  for (int i = 0; i < PF_N; i++) {
    _pf_particles[i].x       += _pf_approx_gaussian() * 1.0f;
    _pf_particles[i].y       += _pf_approx_gaussian() * 1.0f;
    _pf_particles[i].heading  = _pf_wrap_heading(
        _pf_particles[i].heading + _pf_approx_gaussian() * 1.0f);
  }
}

static Pose pf_estimate() {
  Pose est;
  float sum_x = 0, sum_y = 0;
  float sum_sin = 0, sum_cos = 0;

  for (int i = 0; i < PF_N; i++) {
    float w = _pf_particles[i].weight;
    sum_x += w * _pf_particles[i].x;
    sum_y += w * _pf_particles[i].y;
    float h_rad = _pf_particles[i].heading * (float)M_PI / 180.0f;
    sum_sin += w * sin(h_rad);
    sum_cos += w * cos(h_rad);
  }

  est.x = sum_x;
  est.y = sum_y;
  est.heading = atan2(sum_sin, sum_cos) * 180.0f / (float)M_PI;
  if (est.heading < 0) est.heading += 360.0f;
  return est;
}

#endif
