#include <cmath>

#if defined(_WIN32) && defined(__clang__)
extern "C" void sincos(double value, double *sine, double *cosine) {
  *sine = std::sin(value);
  *cosine = std::cos(value);
}
#endif
