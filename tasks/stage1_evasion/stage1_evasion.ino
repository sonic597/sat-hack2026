// stage1_evasion.ino — Space Invader Evasion (Stage 1)
//
// Arena: 1.5m × 0.5m box. Debris launched toward the car every ~5s for 30s.
// Goal: survive by dodging laterally within the box.
//
// Hardware setup for this stage:
//   - Sensor side-mounted, facing the debris launch line (2m away)
//   - Car oriented so wheel-forward points along the 1.5m box axis
//   - Initial pose: heading = 90° so that move_forward → world +X (lateral)
//
// Strategy:
//   SCANNING  – oscillate heading ±SWEEP_MAX_DEG so the side sensor sweeps
//               across the full debris field; read distance each step
//   DODGING   – debris detected → straighten heading, drive laterally to evade
//   RECENTRE  – after dodge, if too close to a box edge, drift back to centre

#include <calibration.h>
#include <hal.h>
#include <movement.h>
#include <sensing.h>
#include <localize.h>

// -------------------------------------------------------------------------
// Stage 1 tuning constants
// -------------------------------------------------------------------------

// Sweep parameters
#define SWEEP_STEP_DEG    5.0f   // heading change per loop step (degrees)
#define SWEEP_MAX_DEG    20.0f   // maximum swing each side of neutral

// Dodge parameters
#define DODGE_SPEED      200     // PWM speed used for all lateral movement
#define DODGE_CM         30.0f   // lateral distance per evasion (cm)

// Box boundary margins (cm from wall)
// If within BOUNDARY_MARGIN of an edge the dodge is forced toward centre.
// If still within RECENTRE_MARGIN after a dodge the car enters RECENTRE state.
#define BOUNDARY_MARGIN  15.0f
#define RECENTRE_MARGIN  20.0f

// Starting lateral position: centre of the 1.5m box (EVASION_W_CM / 2)
#define START_X          75.0f   // cm

// SWEEP_DODGE_SIGN controls which direction correlates with a positive sweep.
// +1 → positive sweep angle (turned left) causes a forward dodge.
// -1 → positive sweep angle (turned left) causes a backward dodge.
// Flip this constant if the sensor is mounted on the opposite side.
#define SWEEP_DODGE_SIGN (-1)

// -------------------------------------------------------------------------
// State machine
// -------------------------------------------------------------------------

enum Stage1State { SCANNING, DODGING, RECENTRE };

static Stage1State s1_state;
static float       s1_sweep_offset;  // net heading offset from neutral (degrees, turn_degrees sign)
static int         s1_sweep_dir;     // +1 or -1
static int         s1_dodge_dir;     // +1 = move_forward (+X), -1 = move_backward (−X)

// -------------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------------

// Pick dodge direction based on boundary priority and sweep angle.
// Boundary always wins; otherwise infer from which part of the field debris came from.
static int s1_choose_dodge() {
    float x = loc_get().x;

    // Hard boundary: force toward centre
    if (x > EVASION_W_CM - BOUNDARY_MARGIN) return -1;  // near right edge → go left
    if (x < BOUNDARY_MARGIN)                return  1;  // near left edge  → go right

    // Infer from sweep direction when debris was detected.
    // sweep_offset > 0 means car turned left (CCW); with side-mounted sensor this
    // sweeps the beam one way, implying debris is on that side of the field.
    return SWEEP_DODGE_SIGN * ((s1_sweep_offset >= 0.0f) ? -1 : 1);
}

// Clamp dodge distance so the car cannot overshoot the boundary margin.
static float s1_clamp_dodge(int dir) {
    float x   = loc_get().x;
    float room = (dir > 0) ? (EVASION_W_CM - BOUNDARY_MARGIN - x)
                           : (x - BOUNDARY_MARGIN);
    if (room <= 0.0f) return 0.0f;
    return (DODGE_CM < room) ? DODGE_CM : room;
}

// -------------------------------------------------------------------------
// Arduino entry points
// -------------------------------------------------------------------------

void setup() {
    Serial.begin(9600);
    Serial.println("[setup] Stage 1 Evasion starting...");

    hal_init();
    loc_reset();

    // Rotate the localize frame so wheel-forward maps to world +X.
    // With heading = 90°:  sin(90°) = 1 → x += cm per move_forward call.
    // This lets loc_get().x track lateral position within the 1.5m box.
    loc_update_turn(90.0f);
    loc_correct(START_X, 0.0f);
    Serial.print("[setup] Initial position set to x="); Serial.print(START_X); Serial.println(" cm");

    s1_state        = SCANNING;
    s1_sweep_offset = 0.0f;
    s1_sweep_dir    = 1;
    s1_dodge_dir    = 1;

    Serial.println("[setup] Done. Entering SCANNING state.");
}

void loop() {
    switch (s1_state) {

    // ------------------------------------------------------------------
    case SCANNING: {
        // Advance the sweep one step and take a sensor reading.
        float step = s1_sweep_dir * SWEEP_STEP_DEG;
        turnLeft(step);       // blocking; also updates localize heading
        s1_sweep_offset += step;
        Serial.print("[SCANNING] sweep_offset="); Serial.print(s1_sweep_offset); Serial.print(" dir="); Serial.println(s1_sweep_dir);

        // Reverse sweep direction at the limits
        if (s1_sweep_offset >= SWEEP_MAX_DEG) {
            s1_sweep_offset =  SWEEP_MAX_DEG;
            s1_sweep_dir    = -1;
            Serial.println("[SCANNING] Hit +limit, reversing sweep direction");
        } else if (s1_sweep_offset <= -SWEEP_MAX_DEG) {
            s1_sweep_offset = -SWEEP_MAX_DEG;
            s1_sweep_dir    =  1;
            Serial.println("[SCANNING] Hit -limit, reversing sweep direction");
        }

        float dist = read_distance();
        Serial.print("[SCANNING] distance="); Serial.print(dist); Serial.print(" cm (threshold="); Serial.print(INCOMING_THRESHOLD); Serial.println(")");

        if (dist < INCOMING_THRESHOLD) {
            Serial.println("[SCANNING] *** DEBRIS DETECTED — transitioning to DODGING ***");

            // Debris incoming — record dodge direction while we still know
            // the sweep angle, then straighten up and evade.
            s1_dodge_dir = s1_choose_dodge();
            Serial.print("[SCANNING] dodge_dir="); Serial.println(s1_dodge_dir);

            // Straighten heading before driving laterally.
            // turn_degrees sign convention: positive = left (CCW).
            // To undo a positive (left) offset we turn right (negative).
            turnRight(s1_sweep_offset);
            Serial.print("[SCANNING] Straightened heading (turned right "); Serial.print(s1_sweep_offset); Serial.println(" deg)");
            s1_sweep_offset = 0.0f;
            s1_sweep_dir    = 1;

            s1_state = DODGING;
        }
        break;
    }

    // ------------------------------------------------------------------
    case DODGING: {
        float cm = s1_clamp_dodge(s1_dodge_dir);
        Serial.print("[DODGING] dodge_dir="); Serial.print(s1_dodge_dir); Serial.print(" clamped_dist="); Serial.print(cm); Serial.println(" cm");

        if (cm > 0.0f) {
            if (s1_dodge_dir > 0) {
                // Forward along wheel axis → +X in localize frame.
                Serial.println("[DODGING] Moving FORWARD");
                forward_dist(cm);
            } else {
                Serial.println("[DODGING] Moving REVERSE");
                reverse_dist(cm);
            }
        } else {
            Serial.println("[DODGING] No room to dodge (clamped to 0)");
        }

        // After the dodge, decide whether to recentre or resume scanning.
        float x = loc_get().x;
        Serial.print("[DODGING] Post-dodge x="); Serial.print(x); Serial.println(" cm");
        if (x > EVASION_W_CM - RECENTRE_MARGIN || x < RECENTRE_MARGIN) {
            Serial.println("[DODGING] Too close to edge — transitioning to RECENTRE");
            s1_state = RECENTRE;
        } else {
            Serial.println("[DODGING] Position OK — resuming SCANNING");
            s1_state = SCANNING;
        }
        break;
    }

    // ------------------------------------------------------------------
    case RECENTRE: {
        float x      = loc_get().x;
        float centre = EVASION_W_CM / 2.0f;   // 75 cm
        float err    = x - centre;
        float cm     = (err < 0.0f) ? -err : err;
        Serial.print("[RECENTRE] x="); Serial.print(x); Serial.print(" err="); Serial.print(err); Serial.print(" cm_to_move="); Serial.println(cm);

        if (cm < 5.0f) {
            // Close enough to centre; resume scanning
            Serial.println("[RECENTRE] Close enough to centre — resuming SCANNING");
            s1_state = SCANNING;
            break;
        }

        if (err > 0.0f) {
            // Too far in +X → move backward (−X)
            Serial.println("[RECENTRE] Too far right — moving REVERSE to centre");
            reverse_dist(cm);
        } else {
            // Too far in −X → move forward (+X)
            Serial.println("[RECENTRE] Too far left — moving FORWARD to centre");
            forward_dist(cm);
        }

        Serial.println("[RECENTRE] Done — resuming SCANNING");
        s1_state = SCANNING;
        break;
    }

    } // end switch
}
