#include "../lib/hal.h"
#include "../lib/movement.h"
#include "../lib/localize.h"
#include "../lib/sensing.h"
#include "../lib/calibration.h"

/*
 * Interactive calibration helper sketch.
 *
 * Send a command character over the serial console to perform one of the
 * calibration steps described in calibration.h. 
 */

void print_help() {
    Serial.println("Calibration helper commands:");
    Serial.println("  t  - trim test (drive forward 2s at REF_SPEED)");
    Serial.println("  d  - distance test (drive forward 3s at REF_SPEED)");
    Serial.println("  r  - rotation test (360 degrees at REF_SPEED)");
    Serial.println("  s  - sensor offset check (press key after positioning)");
    Serial.println("  h  - this help message");
}

void setup() {
    Serial.begin(9600);
    hal_init();
    Serial.println("=== Calibration Helper ===");
    print_help();
}

void loop() {
    if (Serial.available() > 0) {
        char c = Serial.read();
        switch (c) {
            case 't':
                Serial.println("Performing trim test: driving forward 2000 ms...");
                move_forward(REF_SPEED, 2000);
                Serial.println("Done. Measure lateral drift over 2m and adjust TRIM_L/TRIM_R.");
                break;
            case 'd':
                Serial.println("Performing distance test: driving forward 3000 ms...");
                move_forward(REF_SPEED, 3000);
                Serial.println("Done. Measure distance travelled and compute MS_PER_CM.");
                break;
            case 'r':
                Serial.println("Performing rotation test: rotating 360 degrees...");
                turn_degrees(REF_SPEED, 360.0f);
                Serial.println("Done. Measure rotation time and compute MS_PER_DEG.");
                break;
            case 's':
                Serial.println("Sensor offset check.");
                Serial.println("Place the car at a known distance from a wall and press any key.");
                while (Serial.available() == 0) {
                    // wait
                }
                // flush input
                while (Serial.available() > 0) Serial.read();
                {
                    float d = read_distance_world();
                    Serial.print("read_distance_world() = ");
                    Serial.print(d);
                    Serial.println(" cm");
                }
                break;
            case 'h':
                print_help();
                break;
            default:
                // ignore other characters
                break;
        }
    }
}
