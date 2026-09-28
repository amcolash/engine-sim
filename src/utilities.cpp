#include "../include/utilities.h"

#include <cmath>

double modularDistance(double a0, double b0, double mod) {
    double a, b;
    if (a0 < b0) {
        a = a0;
        b = b0;
    }
    else {
        a = b0;
        b = a0;
    }

    return std::fmin(b - a, a + mod - b);
}

double positiveMod(double x, double mod) {
    if (x < 0) {
        x = std::ceil(-x / mod) * mod + x;
    }

    return std::fmod(x, mod);
}

double erfApproximation(double x) {
    const double a1 = 0.278393;
    const double a2 = 0.230389;
    const double a3 = 0.000972;
    const double a4 = 0.078108;

    const double x2 = x * x;
    const double x3 = x2 * x;
    const double x4 = x3 * x;

    const double q = 1 / (1 + a1 * x + a2 * x2 + a3 * x3 + a4 * x4);
    const double q2 = q * q;
    const double q4 = q2 * q2;

    return 1 - q4;
}

namespace rng {
    thread_local uint64_t s_rngState = 0x853c49e6748fea9bULL;

    void seed(uint64_t s) {
        s_rngState = (s == 0) ? 0x853c49e6748fea9bULL : s;
    }

    uint64_t next64() {
        uint64_t z = (s_rngState += 0x9e3779b97f4a7c15ULL);
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        return z ^ (z >> 31);
    }

    double uniform() {
        return (next64() >> 11) * (1.0 / 9007199254740992.0);
    }

    float uniformFloat() {
        return static_cast<float>((next64() >> 40) * (1.0 / 16777216.0));
    }

    float uniformBipolarFloat() {
        return uniformFloat() * 2.0f - 1.0f;
    }
}
