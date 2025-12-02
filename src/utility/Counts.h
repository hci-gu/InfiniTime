#pragma once

#include <cstdint>
#include <cstddef>

namespace Pinetime {
  namespace Utility {
    /**
     * Calculates proprietary "Counts" from accelerometer data.
     * This is a calculation based on a minute of accelerometer data.
     *
     * @param accX Array of X-axis accelerometer values
     * @param accY Array of Y-axis accelerometer values
     * @param accZ Array of Z-axis accelerometer values
     * @param length Number of samples in each array
     * @return The calculated counts value
     */
    float CalculateCounts(const float* accX, const float* accY, const float* accZ, size_t length);
  }
}
