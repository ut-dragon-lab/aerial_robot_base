#pragma once

#include <cmath>
#include <random>

namespace {
unsigned int noise_seed = 0;
}

namespace aerial_robot_simulation {
inline double gaussianKernel(double sigma) {
  static thread_local std::mt19937 gen(std::random_device{}());
  std::normal_distribution<double> dist(0.0, sigma);
  return dist(gen);
}

inline double addNoise(double& current_drift, double drift, double drift_frequency, double offset,
                       double gaussian_noise, double dt) {
  current_drift =
      std::exp(-dt * drift_frequency) * current_drift + dt * gaussianKernel(std::sqrt(2.0 * drift_frequency) * drift);
  return offset + current_drift + gaussianKernel(gaussian_noise);
}

};  // namespace aerial_robot_simulation
