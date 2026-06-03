#pragma once

#include <math.h>

inline double kRHuber(const double& Dmtest, const double& Nsigma = 0.1) {
  if (std::abs(Dmtest) > Nsigma) {
    return 2.0 * Nsigma / Dmtest - (Nsigma * Nsigma) / (Dmtest * Dmtest);
  } else {
    return 1.0;
  }
}