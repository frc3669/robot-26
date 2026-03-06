#include "angleMath.h"

void am::limit(float &angle) {
    while (angle > M_PI){
        angle -= M_PI*2;
    }
    while (angle < -M_PI){
        angle += M_PI*2;
    }
}

void am::limit(double &angle) {
    while (angle > M_PI){
        angle -= M_PI*2;
    }
    while (angle < -M_PI){
        angle += M_PI*2;
    }
}

void am::limitDegrees(double &angle) {
    while (angle > 180){
        angle -= 360;
    }
    while (angle < -180){
        angle += 360;
    }
}

void am::limit(units::radian_t &angle) {
    while (angle > 180_deg) {
        angle -= 360_deg;
    }
    while (angle < -180_deg) {
        angle += 360_deg;
    }
}

void am::limit(units::degree_t &angle) {
    while (angle > 180_deg) {
        angle -= 360_deg;
    }
    while (angle < -180_deg) {
        angle += 360_deg;
    }
}