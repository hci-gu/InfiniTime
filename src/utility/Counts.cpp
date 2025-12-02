#include "utility/Counts.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace Pinetime {
  namespace Utility {
    namespace {
      // Filter initial response for 8th order filter
      constexpr float zi[9] = {
        -0.07532883864659122f, -0.0753546620840857f, 0.22603070565973946f,
        0.22598432569253424f, -0.22599275859607648f, -0.22600915159543014f,
        0.07533559105142006f, 0.07533478851866574f, 0.0f
      };

      // Zero initial conditions for 20th order filter
      constexpr float zi_zeros[21] = {0};

      // Filter coefficients for 8th order filter (filterAB2)
      constexpr float b2[9] = {
        0.0753349039750657f, 0.0f, -0.3013396159002628f, 0.0f,
        0.4520094238503942f, 0.0f, -0.3013396159002628f, 0.0f,
        0.0753349039750657f
      };

      constexpr float a2[9] = {
        1.0f, -4.2575497111306f, 7.543176557521139f, -7.64673626503976f,
        5.33187310787808f, -2.7027389238066353f, 0.8452545660100996f,
        -0.1323148049950936f, 0.019035473586069288f
      };

      // Filter coefficients for 20th order filter (filterAB)
      constexpr float b1[21] = {
        0.047390185f, -0.1185406f, 0.13853539999999998f, -0.10874584999999999f,
        0.05192086f, -0.01952195f, 0.0061545770000000005f, 0.017865045f,
        -0.03681861f, 0.047021555f, -0.050736804999999996f, 0.046172355f,
        -0.044404475f, 0.035013095f, -0.012522805f, -0.0044642829999999994f,
        0.012385774999999998f, -0.009048032999999999f, 0.0033278025f,
        -0.0007813798f, -0.00018936195f
      };

      constexpr float a1[21] = {
        1.0f, -4.1637f, 7.5712f, -7.9805f, 5.385f, -2.4636f, 0.89238f,
        0.06361f, -1.3481f, 2.4734f, -2.9257f, 2.9298f, -2.7816f,
        2.4777f, -1.6847f, 0.46483f, 0.46565f, -0.67312f, 0.4162f,
        -0.13832f, 0.019852f
      };

      // Constants for counts calculation
      constexpr float deadband = 0.068f;
      constexpr float peakThreshold = 2.13f;
      constexpr float adcResolution = 0.0164f;
      constexpr int integN = 10;

      /**
       * Odd extension of signal for edge handling in filtfilt
       */
      void oddExt(const float* x, size_t xLen, int n, float* result) {
        // Left extension: 2*x[0] - x[n-1-i] for i in [0, n)
        for (int i = 0; i < n; i++) {
          result[i] = 2.0f * x[0] - x[n - 1 - i];
        }

        // Copy original data
        std::memcpy(result + n, x, xLen * sizeof(float));

        // Right extension: 2*x[xLen-1] - x[xLen-1-n+i] for i in [0, n)
        for (int i = 0; i < n; i++) {
          result[xLen + n + i] = 2.0f * x[xLen - 1] - x[xLen - 1 - n + i];
        }
      }

      /**
       * Forward IIR filter (8th order)
       */
      void filterAB2Forward(float* data, size_t length) {
        float z[9];
        for (int i = 0; i < 9; i++) {
          z[i] = zi[i] * data[0];
        }

        float x0[9];
        for (int i = 0; i < 9 && i < static_cast<int>(length); i++) {
          x0[i] = data[i];
        }

        // Initial samples (i < 9)
        for (int i = 0; i < 9 && i < static_cast<int>(length); i++) {
          if (i >= 8) z[7] = 0.0753349039750657f * x0[i - 8] - 0.019035473586069288f * data[i - 8] + z[8];
          if (i >= 7) z[6] = 0.1323148049950936f * data[i - 7] + z[7];
          if (i >= 6) z[5] = -0.3013396159002628f * x0[i - 6] - 0.8452545660100996f * data[i - 6] + z[6];
          if (i >= 5) z[4] = 2.7027389238066353f * data[i - 5] + z[5];
          if (i >= 4) z[3] = 0.4520094238503942f * x0[i - 4] - 5.33187310787808f * data[i - 4] + z[4];
          if (i >= 3) z[2] = 7.64673626503976f * data[i - 3] + z[3];
          if (i >= 2) z[1] = -0.3013396159002628f * x0[i - 2] - 7.543176557521139f * data[i - 2] + z[2];
          if (i >= 1) z[0] = 4.2575497111306f * data[i - 1] + z[1];

          data[i] = 0.0753349039750657f * data[i] + z[0];
        }

        // Main loop (i >= 9)
        float yi8 = data[0], yi7 = data[1], yi6 = data[2], yi5 = data[3];
        float yi4 = data[4], yi3 = data[5], yi2 = data[6], yi1 = data[7];
        float xi8 = x0[0], xi7 = x0[1], xi6 = x0[2], xi5 = x0[3];
        float xi4 = x0[4], xi3 = x0[5], xi2 = x0[6], xi1 = x0[7];

        for (size_t i = 9; i < length; i++) {
          float za = z[8];
          za = 0.0753349039750657f * xi8 - 0.019035473586069288f * yi8 + za;
          za = 0.1323148049950936f * yi7 + za;
          za = -0.3013396159002628f * xi6 - 0.8452545660100996f * yi6 + za;
          za = 2.7027389238066353f * yi5 + za;
          za = 0.4520094238503942f * xi4 - 5.33187310787808f * yi4 + za;
          za = 7.64673626503976f * yi3 + za;
          za = -0.3013396159002628f * xi2 - 7.543176557521139f * yi2 + za;
          za = 4.2575497111306f * yi1 + za;

          yi8 = yi7; yi7 = yi6; yi6 = yi5; yi5 = yi4;
          yi4 = yi3; yi3 = yi2; yi2 = yi1;
          yi1 = 0.0753349039750657f * data[i] + za;

          xi8 = xi7; xi7 = xi6; xi6 = xi5; xi5 = xi4;
          xi4 = xi3; xi3 = xi2; xi2 = xi1;
          xi1 = data[i];

          data[i] = yi1;
        }
      }

      /**
       * Reverse IIR filter (8th order)
       */
      void filterAB2Reverse(float* data, size_t length) {
        float z[9];
        for (int i = 0; i < 9; i++) {
          z[i] = zi[i] * data[length - 1];
        }

        float x0[9];
        for (int i = 0; i < 9; i++) {
          x0[i] = data[length - 9 + i];
        }

        // Initial samples from the end
        for (int i = 0; i < 9; i++) {
          int idx = static_cast<int>(length) - 1 - i;
          if (i >= 8) z[7] = 0.0753349039750657f * x0[8 - i + 8] - 0.019035473586069288f * data[idx + 8] + z[8];
          if (i >= 7) z[6] = 0.1323148049950936f * data[idx + 7] + z[7];
          if (i >= 6) z[5] = -0.3013396159002628f * x0[8 - i + 6] - 0.8452545660100996f * data[idx + 6] + z[6];
          if (i >= 5) z[4] = 2.7027389238066353f * data[idx + 5] + z[5];
          if (i >= 4) z[3] = 0.4520094238503942f * x0[8 - i + 4] - 5.33187310787808f * data[idx + 4] + z[4];
          if (i >= 3) z[2] = 7.64673626503976f * data[idx + 3] + z[3];
          if (i >= 2) z[1] = -0.3013396159002628f * x0[8 - i + 2] - 7.543176557521139f * data[idx + 2] + z[2];
          if (i >= 1) z[0] = 4.2575497111306f * data[idx + 1] + z[1];

          data[idx] = 0.0753349039750657f * x0[8 - i] + z[0];
        }

        // Main loop in reverse
        float yi8 = data[length - 1], yi7 = data[length - 2], yi6 = data[length - 3], yi5 = data[length - 4];
        float yi4 = data[length - 5], yi3 = data[length - 6], yi2 = data[length - 7], yi1 = data[length - 8];
        float xi8 = x0[8], xi7 = x0[7], xi6 = x0[6], xi5 = x0[5];
        float xi4 = x0[4], xi3 = x0[3], xi2 = x0[2], xi1 = x0[1];

        for (size_t i = 9; i < length; i++) {
          size_t idx = length - 1 - i;
          float za = z[8];
          za = 0.0753349039750657f * xi8 - 0.019035473586069288f * yi8 + za;
          za = 0.1323148049950936f * yi7 + za;
          za = -0.3013396159002628f * xi6 - 0.8452545660100996f * yi6 + za;
          za = 2.7027389238066353f * yi5 + za;
          za = 0.4520094238503942f * xi4 - 5.33187310787808f * yi4 + za;
          za = 7.64673626503976f * yi3 + za;
          za = -0.3013396159002628f * xi2 - 7.543176557521139f * yi2 + za;
          za = 4.2575497111306f * yi1 + za;

          yi8 = yi7; yi7 = yi6; yi6 = yi5; yi5 = yi4;
          yi4 = yi3; yi3 = yi2; yi2 = yi1;
          yi1 = 0.0753349039750657f * data[idx] + za;

          xi8 = xi7; xi7 = xi6; xi6 = xi5; xi5 = xi4;
          xi4 = xi3; xi3 = xi2; xi2 = xi1;
          xi1 = data[idx];

          data[idx] = yi1;
        }
      }

      /**
       * Forward-backward (zero-phase) filtering with 8th order filter
       */
      void filtfilt(const float* input, size_t inputLen, float* output) {
        constexpr int edge = 27;
        size_t extLen = inputLen + 2 * edge;

        // Allocate extended buffer
        float* extended = new float[extLen];

        // Odd extension
        oddExt(input, inputLen, edge, extended);

        // Forward filter
        filterAB2Forward(extended, extLen);

        // Reverse filter
        filterAB2Reverse(extended, extLen);

        // Copy result (remove padding)
        std::memcpy(output, extended + edge, inputLen * sizeof(float));

        delete[] extended;
      }

      /**
       * Forward IIR filter (20th order)
       */
      void filterAB(float* data, size_t length) {
        float z[21] = {0};

        float x0[21];
        for (int i = 0; i < 21 && i < static_cast<int>(length); i++) {
          x0[i] = data[i];
        }

        // Initial samples (i < 21)
        for (int i = 0; i < 21 && i < static_cast<int>(length); i++) {
          if (i >= 20) z[19] = -0.00018936195f * x0[i - 20] - 0.019852f * data[i - 20] + z[20];
          if (i >= 19) z[18] = -0.0007813798f * x0[i - 19] + 0.13832f * data[i - 19] + z[19];
          if (i >= 18) z[17] = 0.0033278025f * x0[i - 18] - 0.4162f * data[i - 18] + z[18];
          if (i >= 17) z[16] = -0.009048032999999999f * x0[i - 17] + 0.67312f * data[i - 17] + z[17];
          if (i >= 16) z[15] = 0.012385774999999998f * x0[i - 16] - 0.46565f * data[i - 16] + z[16];
          if (i >= 15) z[14] = -0.0044642829999999994f * x0[i - 15] - 0.46483f * data[i - 15] + z[15];
          if (i >= 14) z[13] = -0.012522805f * x0[i - 14] + 1.6847f * data[i - 14] + z[14];
          if (i >= 13) z[12] = 0.035013095f * x0[i - 13] - 2.4777f * data[i - 13] + z[13];
          if (i >= 12) z[11] = -0.044404475f * x0[i - 12] + 2.7816f * data[i - 12] + z[12];
          if (i >= 11) z[10] = 0.046172355f * x0[i - 11] - 2.9298f * data[i - 11] + z[11];
          if (i >= 10) z[9] = -0.050736804999999996f * x0[i - 10] + 2.9257f * data[i - 10] + z[10];
          if (i >= 9) z[8] = 0.047021555f * x0[i - 9] - 2.4734f * data[i - 9] + z[9];
          if (i >= 8) z[7] = -0.03681861f * x0[i - 8] + 1.3481f * data[i - 8] + z[8];
          if (i >= 7) z[6] = 0.017865045f * x0[i - 7] - 0.06361f * data[i - 7] + z[7];
          if (i >= 6) z[5] = 0.0061545770000000005f * x0[i - 6] - 0.89238f * data[i - 6] + z[6];
          if (i >= 5) z[4] = -0.01952195f * x0[i - 5] + 2.4636f * data[i - 5] + z[5];
          if (i >= 4) z[3] = 0.05192086f * x0[i - 4] - 5.385f * data[i - 4] + z[4];
          if (i >= 3) z[2] = -0.10874584999999999f * x0[i - 3] + 7.9805f * data[i - 3] + z[3];
          if (i >= 2) z[1] = 0.13853539999999998f * x0[i - 2] - 7.5712f * data[i - 2] + z[2];
          if (i >= 1) z[0] = -0.1185406f * x0[i - 1] + 4.1637f * data[i - 1] + z[1];

          data[i] = 0.047390185f * x0[i] + z[0];
        }

        // Main loop (i >= 21)
        float yi20 = data[0], yi19 = data[1], yi18 = data[2], yi17 = data[3];
        float yi16 = data[4], yi15 = data[5], yi14 = data[6], yi13 = data[7];
        float yi12 = data[8], yi11 = data[9], yi10 = data[10], yi9 = data[11];
        float yi8 = data[12], yi7 = data[13], yi6 = data[14], yi5 = data[15];
        float yi4 = data[16], yi3 = data[17], yi2 = data[18], yi1 = data[19];

        float xi20 = x0[0], xi19 = x0[1], xi18 = x0[2], xi17 = x0[3];
        float xi16 = x0[4], xi15 = x0[5], xi14 = x0[6], xi13 = x0[7];
        float xi12 = x0[8], xi11 = x0[9], xi10 = x0[10], xi9 = x0[11];
        float xi8 = x0[12], xi7 = x0[13], xi6 = x0[14], xi5 = x0[15];
        float xi4 = x0[16], xi3 = x0[17], xi2 = x0[18], xi1 = x0[19];

        for (size_t i = 21; i < length; i++) {
          float za = z[20];

          za = -0.00018936195f * xi20 - 0.019852f * yi20 + za;
          za = -0.0007813798f * xi19 + 0.13832f * yi19 + za;
          za = 0.0033278025f * xi18 - 0.4162f * yi18 + za;
          za = -0.009048032999999999f * xi17 + 0.67312f * yi17 + za;
          za = 0.012385774999999998f * xi16 - 0.46565f * yi16 + za;
          za = -0.0044642829999999994f * xi15 - 0.46483f * yi15 + za;
          za = -0.012522805f * xi14 + 1.6847f * yi14 + za;
          za = 0.035013095f * xi13 - 2.4777f * yi13 + za;
          za = -0.044404475f * xi12 + 2.7816f * yi12 + za;
          za = 0.046172355f * xi11 - 2.9298f * yi11 + za;
          za = -0.050736804999999996f * xi10 + 2.9257f * yi10 + za;
          za = 0.047021555f * xi9 - 2.4734f * yi9 + za;
          za = -0.03681861f * xi8 + 1.3481f * yi8 + za;
          za = 0.017865045f * xi7 - 0.06361f * yi7 + za;
          za = 0.0061545770000000005f * xi6 - 0.89238f * yi6 + za;
          za = -0.01952195f * xi5 + 2.4636f * yi5 + za;
          za = 0.05192086f * xi4 - 5.385f * yi4 + za;
          za = -0.10874584999999999f * xi3 + 7.9805f * yi3 + za;
          za = 0.13853539999999998f * xi2 - 7.5712f * yi2 + za;
          za = -0.1185406f * xi1 + 4.1637f * yi1 + za;

          yi20 = yi19; yi19 = yi18; yi18 = yi17; yi17 = yi16;
          yi16 = yi15; yi15 = yi14; yi14 = yi13; yi13 = yi12;
          yi12 = yi11; yi11 = yi10; yi10 = yi9; yi9 = yi8;
          yi8 = yi7; yi7 = yi6; yi6 = yi5; yi5 = yi4;
          yi4 = yi3; yi3 = yi2; yi2 = yi1;
          yi1 = 0.047390185f * data[i] + za;

          xi20 = xi19; xi19 = xi18; xi18 = xi17; xi17 = xi16;
          xi16 = xi15; xi15 = xi14; xi14 = xi13; xi13 = xi12;
          xi12 = xi11; xi11 = xi10; xi10 = xi9; xi9 = xi8;
          xi8 = xi7; xi7 = xi6; xi6 = xi5; xi5 = xi4;
          xi4 = xi3; xi3 = xi2; xi2 = xi1;
          xi1 = data[i];

          data[i] = yi1;
        }
      }

      /**
       * Running sum with threshold
       */
      float runsum(const float* data, size_t dataLen, int length, float threshold) {
        float total = 0.0f;
        size_t cnt = (dataLen + length - 1) / length;

        for (size_t n = 0; n < cnt; n++) {
          float rs = 0.0f;
          for (int p = length * n; p < length * (n + 1); p++) {
            if (static_cast<size_t>(p) < dataLen && data[p] >= threshold) {
              rs += data[p] - threshold;
            }
          }
          total += rs;
        }

        return total;
      }

      /**
       * Get counts for a single axis
       */
      float getCounts(const float* values, size_t length) {
        // Allocate working buffer
        float* filtered = new float[length];

        // Apply filtfilt (forward-backward 8th order filter)
        filtfilt(values, length, filtered);

        // Apply 20th order filter
        filterAB(filtered, length);

        // Downsample by 3, clip to peak threshold, apply deadband and quantize
        size_t downsampledLen = (length + 2) / 3;
        float* processed = new float[downsampledLen];

        size_t j = 0;
        for (size_t i = 0; i < length; i += 3) {
          float d = filtered[i];

          // Clip to peak threshold
          if (d < -peakThreshold) d = -peakThreshold;
          else if (d > peakThreshold) d = peakThreshold;

          // Apply deadband and quantize
          float absD = std::fabs(d);
          if (absD < deadband) {
            processed[j] = 0.0f;
          } else {
            processed[j] = std::floor(absD / adcResolution);
          }
          j++;
        }

        // Calculate running sum
        float result = runsum(processed, j, integN, 0.0f);

        delete[] filtered;
        delete[] processed;

        return result;
      }
    } // anonymous namespace

    float CalculateCounts(const float* accX, const float* accY, const float* accZ, size_t length) {
      float x = getCounts(accX, length);
      float y = getCounts(accY, length);
      float z = getCounts(accZ, length);

      return std::sqrt(x * x + y * y + z * z);
    }
  } // namespace Utility
} // namespace Pinetime
