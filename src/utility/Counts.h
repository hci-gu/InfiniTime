#pragma once

#include <cstdint>
#include <cstddef>

namespace Pinetime {
  namespace Utility {
    /**
     * Calculates proprietary "Counts" from accelerometer data.
     * This is a calculation based on a minute of accelerometer data.
     * 
     * Uses minimal static buffers, processes one axis at a time.
     * NOT thread-safe - only call from one task at a time.
     */
    class CountsCalculator {
    public:
      // Maximum supported input length (10Hz * 60s)
      static constexpr size_t maxInputLength = 600;
      
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
      // Process a single axis and return its counts contribution
      static float ProcessAxis(const int16_t* rawData, size_t length);
      
      // Static working buffers - minimal set for single-axis processing
      // workBuffer is used for: int16_t->float conversion, filtfilt extended data
      static constexpr size_t extendedLength = maxInputLength + 54; // filtfilt edge extension
      static constexpr size_t downsampledLength = (maxInputLength + 2) / 3;
      
      static float workBuffer[extendedLength];       // Main work buffer
      static float downsampleBuffer[downsampledLength]; // For downsampled data
    };
  }
}
