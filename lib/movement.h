#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <ADCS.h>
#include "calibration.h"

const int STARTUP_MS = 0;    // y-intercept

void stopMotors() {
    motorL(0, 0);
    motorR(0, 0);
    Serial.println("[movement] Motors stopped");
}

// raw functions
void forward(int speed) {
    int r = speed - offset;
    if (r < 0) r = 0;
    Serial.print("[movement] forward speed="); Serial.print(speed); Serial.print(" r_adjusted="); Serial.println(r);
    motorL(speed, 0);
    motorR(r, 0);
}

void reverse(int speed) {
    int r = speed - offset;
    if (r < 0) r = 0;
    Serial.print("[movement] reverse speed="); Serial.print(speed); Serial.print(" r_adjusted="); Serial.println(r);
    motorL(0, speed);
    motorR(0, r);
}

void forward_dist(float cm) {
    if (cm <= 0) return;

    // Calculate time using y = mx + c
    unsigned long duration = (cm * MS_PER_CM) + STARTUP_MS;
    Serial.print("[movement] forward_dist cm="); Serial.print(cm); Serial.print(" duration="); Serial.print(duration); Serial.println(" ms");
    forward(REF_SPEED);
    delay(duration);
    stopMotors();
    delay(100);
}

void reverse_dist(float cm) {
    if (cm <= 0) return;

    unsigned long duration = (cm * MS_PER_CM) + STARTUP_MS;
    Serial.print("[movement] reverse_dist cm="); Serial.print(cm); Serial.print(" duration="); Serial.print(duration); Serial.println(" ms");
    reverse(REF_SPEED);
    delay(duration);
    stopMotors();
    delay(100);
}

void turnLeft(int degrees) {
    unsigned long duration = degrees * MS_PER_DEG;
    Serial.print("[movement] turnLeft deg="); Serial.print(degrees); Serial.print(" duration="); Serial.print(duration); Serial.println(" ms");
    motorL(0, REF_SPEED);
    motorR(REF_SPEED - offset, 0);
    delay(duration);
    stopMotors();
}

void turnRight(int degrees) {
    unsigned long duration = degrees * MS_PER_DEG;
    Serial.print("[movement] turnRight deg="); Serial.print(degrees); Serial.print(" duration="); Serial.print(duration); Serial.println(" ms");
    motorL(REF_SPEED, 0);
    motorR(0, REF_SPEED - offset);
    delay(duration);
    stopMotors();
}

#endif