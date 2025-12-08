#pragma once

#include <cstdint>
#include <cstddef>
#include <FreeRTOS.h>

namespace Pinetime {
  namespace Utility {
    /**
     * Calculates proprietary "Counts" from accelerometer data.
     * This is a calculation based on a minute of accelerometer data.
     * The input is expected at 10Hz; the algorithm internally resamples to 30Hz
     * to match the original filter design.
     * 
     * Uses minimal static buffers, processes one axis at a time.
     * NOT thread-safe - only call from one task at a time.
     */
    class CountsCalculator {
    public:
      // Maximum supported input length (10Hz * 60s)
      static constexpr size_t maxInputLength = 600;
      static constexpr size_t resampleFactor = 3;             // Up-sample 10Hz -> 30Hz
      static constexpr size_t maxResampledLength = maxInputLength * resampleFactor;
      
      /**
       * Calculate counts from int16_t accelerometer data.
       * Data is expected in binary milli-g format (1g = 1024).
       * Input arrays must be linearized (not circular).
       * 
       * @param rawX Array of X-axis values in binary milli-g
       * @param rawY Array of Y-axis values in binary milli-g
       * @param rawZ Array of Z-axis values in binary milli-g
       * @param length Number of samples (must be <= maxInputLength)
       * @return The calculated counts value, or 0 if length > maxInputLength
       */
      static float Calculate(const int16_t* rawX, const int16_t* rawY, const int16_t* rawZ, size_t length);
      
    private:
      // Process a single axis (input already converted to g units)
      static float ProcessAxis(const float* scaledData, size_t length);
      
      // Static working buffers - minimal set for single-axis processing
      // workBuffer holds resampled data plus filtfilt edge extension
      static constexpr size_t extendedLength = maxResampledLength + 54; // filtfilt edge extension (27 * 2)
      static constexpr size_t downsampledLength = (maxResampledLength + resampleFactor - 1) / resampleFactor;
      
      static float* workBuffer;                // Main work buffer (allocated on demand)
      static float* downsampleBuffer;          // Downsampled data / temp scaling buffer (allocated on demand)
      static bool AllocBuffers();
      static void FreeBuffers();
    };
  }
}
