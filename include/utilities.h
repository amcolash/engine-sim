#ifndef ATG_ENGINE_SIM_UTILITIES_H
#define ATG_ENGINE_SIM_UTILITIES_H

double modularDistance(double a, double b, double mod = 1.0);
double positiveMod(double x, double mod);
double erfApproximation(double x);

template <typename t>
inline t clamp(t x, t x0 = static_cast<t>(0.0), t x1 = static_cast<t>(1.0)) {
    if (x <= x0) return x0;
    else if (x >= x1) return x1;
    else return x;
}

#include <cstdint>

namespace rng {
    void seed(uint64_t s);
    uint64_t next64();
    double uniform();             // [0.0, 1.0)
    float uniformFloat();         // [0.0f, 1.0f)
    float uniformBipolarFloat();  // [-1.0f, 1.0f)
}

#endif /* ATG_ENGINE_SIM_UTILITIES_H */
