#pragma once
#include <math.h>
#include <complex.h>
#include <units/angle.h>

using namespace std;

namespace am{
    void limit(float &angle);
    void limit(double &angle);
    void limit(units::radian_t &angle);
    void limit(units::degree_t &angle);
    void limitDegrees(double &angle);
    float getProjectionSize(complex<float> a, complex<float> b);
}