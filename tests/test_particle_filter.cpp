// Desktop test for particle_filter.h
// Compile: g++ -std=c++11 -o test_pf tests/test_particle_filter.cpp -lm

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

// Mock Arduino random() for desktop
long random(long min, long max) {
  return min + (rand() % (max - min));
}

#include "../lib/particle_filter.h"

static int tests_passed = 0;
static int tests_failed = 0;

#define CHECK(cond, msg) do { \
  if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
  else      { tests_failed++; printf("  FAIL: %s\n", msg); } \
} while(0)

// ---------- helpers ----------

// Build a grid: 30 wide x 60 tall, 10cm cells, all FREE (0)
static uint8_t grid[450]; // 30*60 / 4 = 450 bytes

static void grid_clear() {
  memset(grid, 0, sizeof(grid));
}

static void grid_set(int gx, int gy, uint8_t val) {
  int idx = gy * 30 + gx;
  int byte_idx = idx / 4;
  int shift = (idx % 4) * 2;
  grid[byte_idx] &= ~(0x03 << shift);
  grid[byte_idx] |= (val & 0x03) << shift;
}

// ---------- tests ----------

void test_ray_cast_empty() {
  printf("Test: ray cast on empty grid\n");
  grid_clear();
  pf_set_map(grid, 30, 60, 10);

  // Cast straight ahead (+Y direction) from center
  float d = _pf_ray_cast(150.0f, 100.0f, 0.0f); // heading=0 rad → +Y
  CHECK(d >= PF_MAX_RANGE - 5.0f, "empty grid returns ~max range");
}

void test_ray_cast_wall() {
  printf("Test: ray cast hitting wall\n");
  grid_clear();
  pf_set_map(grid, 30, 60, 10);

  // Place a wall across full width at row 20 (y=200..210 cm)
  for (int x = 0; x < 30; x++) {
    grid_set(x, 20, 2); // WALL
  }

  // Cast from (150, 100) heading 0 rad (+Y). Wall at y~200 → dist ~100cm
  float d = _pf_ray_cast(150.0f, 100.0f, 0.0f);
  CHECK(d > 90.0f && d < 110.0f,
        "wall at y=200, start y=100 → distance ~100cm");
}

void test_grid_get_oob() {
  printf("Test: out-of-bounds grid access returns WALL\n");
  grid_clear();
  pf_set_map(grid, 30, 60, 10);

  CHECK(_pf_grid_get(-1, 0) == 2, "negative x → WALL");
  CHECK(_pf_grid_get(0, -1) == 2, "negative y → WALL");
  CHECK(_pf_grid_get(30, 0) == 2, "x=width → WALL");
  CHECK(_pf_grid_get(0, 60) == 2, "y=height → WALL");
  CHECK(_pf_grid_get(0, 0) == 0, "valid cell → FREE");
}

void test_convergence() {
  printf("Test: particle convergence toward correct position\n");
  srand(42);
  grid_clear();
  pf_set_map(grid, 30, 60, 10);

  // Wall across full width at row 20 (y=200 cm)
  for (int x = 0; x < 30; x++) {
    grid_set(x, 20, 2);
  }

  // Robot is at (150, 100), heading 0 (facing +Y)
  // Sensor is at body origin for simplicity
  pf_set_sensor_offset(0, 0);

  // Spread particles widely around correct position
  pf_init(150.0f, 100.0f, 0.0f, 20.0f, 10.0f);

  // True sensor reading: wall at y=200, robot at y=100 → ~100cm
  for (int iter = 0; iter < 5; iter++) {
    pf_update(100.0f);
  }

  Pose est = pf_estimate();
  float err_x = fabs(est.x - 150.0f);
  float err_y = fabs(est.y - 100.0f);
  printf("    estimate: (%.1f, %.1f, %.1f°)  err: (%.1f, %.1f)\n",
         est.x, est.y, est.heading, err_x, err_y);

  CHECK(err_y < 25.0f, "y estimate within 25cm of truth after 5 updates");
  CHECK(err_x < 30.0f, "x estimate within 30cm of truth");
}

void test_heading_circular_mean() {
  printf("Test: circular mean heading near 0/360 boundary\n");
  srand(99);
  grid_clear();
  pf_set_map(grid, 30, 60, 10);
  pf_set_sensor_offset(0, 0);

  // Manually set particles near 0° (wrapping)
  float headings[] = {350, 355, 0, 5, 10, 352, 358, 1, 3, 7,
                      348, 356, 359, 2, 6};
  for (int i = 0; i < PF_N; i++) {
    _pf_particles[i].x = 150.0f;
    _pf_particles[i].y = 100.0f;
    _pf_particles[i].heading = headings[i];
    _pf_particles[i].weight = 1.0f / PF_N;
  }

  Pose est = pf_estimate();
  // Should be near 0° (could be ~359° or ~1°)
  float h = est.heading;
  float err = fmin(fabs(h - 0.0f), fabs(h - 360.0f));
  printf("    heading estimate: %.1f°  err from 0°: %.1f°\n", h, err);
  CHECK(err < 10.0f, "circular mean near 0° for particles around 350-10°");
}

void test_outlier_tolerance() {
  printf("Test: outlier reading tolerance (wall gap simulation)\n");
  srand(77);
  grid_clear();
  pf_set_map(grid, 30, 60, 10);

  // Wall at row 20 (y=200cm)
  for (int x = 0; x < 30; x++) {
    grid_set(x, 20, 2);
  }

  pf_set_sensor_offset(0, 0);

  // Set dead-reckoning to correct position (realistic: DR tracks alongside PF)
  loc_correct(150.0f, 100.0f);

  pf_init(150.0f, 100.0f, 0.0f, 5.0f, 3.0f);

  // First converge with good readings
  for (int i = 0; i < 5; i++) {
    pf_update(100.0f);
  }

  Pose before = pf_estimate();
  printf("    before outlier: (%.1f, %.1f)\n", before.x, before.y);

  // Inject bad reading: sensor says 250cm (as if through a wall gap)
  pf_update(250.0f);

  Pose after = pf_estimate();
  float drift = sqrt((after.x - before.x) * (after.x - before.x) +
                      (after.y - before.y) * (after.y - before.y));
  printf("    after outlier:  (%.1f, %.1f)  drift=%.1f\n",
         after.x, after.y, drift);

  CHECK(drift < 20.0f,
        "single outlier reading doesn't cause large drift (<20cm)");
}

void test_oob_particle_elimination() {
  printf("Test: out-of-bounds particle gets eliminated\n");
  srand(123);
  grid_clear();
  pf_set_map(grid, 30, 60, 10);

  // Wall at row 20
  for (int x = 0; x < 30; x++) {
    grid_set(x, 20, 2);
  }

  pf_set_sensor_offset(0, 0);
  pf_init(150.0f, 100.0f, 0.0f, 5.0f, 3.0f);

  // Force one particle way out of bounds
  _pf_particles[0].x = -500.0f;
  _pf_particles[0].y = -500.0f;

  pf_update(100.0f);

  Pose est = pf_estimate();
  // OOB particle should have been resampled away
  CHECK(est.x > 100.0f && est.x < 200.0f,
        "estimate stays in valid range after OOB particle eliminated");
}

void test_skip_extreme_readings() {
  printf("Test: extreme readings are skipped\n");
  srand(55);
  grid_clear();
  pf_set_map(grid, 30, 60, 10);
  pf_set_sensor_offset(0, 0);
  pf_init(150.0f, 100.0f, 0.0f, 5.0f, 3.0f);

  Pose before = pf_estimate();

  // These should be no-ops (early return)
  pf_update(1.0f);    // below minimum
  pf_update(0.0f);
  pf_update(299.5f);  // above max - 1
  pf_update(300.0f);

  Pose after = pf_estimate();
  float drift = sqrt((after.x - before.x) * (after.x - before.x) +
                      (after.y - before.y) * (after.y - before.y));
  // Only roughening jitter should cause tiny changes — but since update
  // returns early, there should be NO change at all
  CHECK(drift < 0.01f, "extreme readings cause no position change");
}

void test_predict() {
  printf("Test: predict forward and turn\n");
  srand(200);
  grid_clear();
  pf_set_map(grid, 30, 60, 10);
  pf_set_sensor_offset(0, 0);

  // All particles at same position, heading 0 (facing +Y)
  for (int i = 0; i < PF_N; i++) {
    _pf_particles[i].x = 100.0f;
    _pf_particles[i].y = 100.0f;
    _pf_particles[i].heading = 0.0f;
    _pf_particles[i].weight = 1.0f / PF_N;
  }

  pf_predict_forward(50.0f, 0.0f); // no noise, pure forward
  Pose est = pf_estimate();
  // heading=0° → x += 50*sin(0)=0, y += 50*cos(0)=50
  printf("    after forward 50: (%.1f, %.1f)\n", est.x, est.y);
  CHECK(fabs(est.y - 150.0f) < 2.0f, "forward 50cm moves y by ~50");
  CHECK(fabs(est.x - 100.0f) < 2.0f, "forward 50cm doesn't change x");

  pf_predict_turn(90.0f, 0.0f); // no noise, pure turn
  est = pf_estimate();
  printf("    after turn 90: heading=%.1f°\n", est.heading);
  CHECK(fabs(est.heading - 90.0f) < 2.0f, "turn 90° sets heading to ~90°");
}

// ---------- main ----------

int main() {
  printf("=== Particle Filter Tests ===\n\n");

  test_grid_get_oob();
  test_ray_cast_empty();
  test_ray_cast_wall();
  test_predict();
  test_convergence();
  test_heading_circular_mean();
  test_outlier_tolerance();
  test_oob_particle_elimination();
  test_skip_extreme_readings();

  printf("\n=== Results: %d passed, %d failed ===\n",
         tests_passed, tests_failed);
  return tests_failed > 0 ? 1 : 0;
}
