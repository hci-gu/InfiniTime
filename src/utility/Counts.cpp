#include "utility/Counts.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <FreeRTOS.h>

namespace Pinetime {
  namespace Utility {
    
    namespace {
      // Filter initial response for 8th order filter
      constexpr float zi[9] = {
        -0.07532883864659122f, -0.0753546620840857f, 0.22603070565973946f,
        0.22598432569253424f, -0.22599275859607648f, -0.22600915159543014f,
        0.07533559105142006f, 0.07533478851866574f, 0.0f
      };

      // Constants for counts calculation
      constexpr float deadband = 0.068f;
      constexpr float peakThreshold = 2.13f;
      constexpr float adcResolution = 0.0164f;
      // Integration window for 10 samples (1s at 10Hz after downsampling)
      constexpr int integN = 10;
      constexpr int filtfiltEdge = 27;
      // Convert raw accelerometer values to g units (BMA421: 1g = 1024)
      constexpr float scale = 1.0f / 1024.0f;
      // Gain applied to the Brond 20th order filter output (matches reference)
      constexpr float brondGain = 0.965f;

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
      // filterAB2Reverse removed (unused)

      /**
       * Forward IIR filter (20th order)
       */
      void filterAB(float* data, size_t length) {
        float z[21] = {0};

        float x0[21];
        for (int i = 0; i < 21 && i < static_cast<int>(length); i++) {
          x0[i] = data[i];
        }

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
          for (size_t p = length * n; p < length * (n + 1) && p < dataLen; p++) {
            if (data[p] >= threshold) {
              rs += data[p] - threshold;
            }
          }
          total += rs;
        }
        return total;
      }
    } // anonymous namespace

    // Static buffer pointers (lazy allocated)
    float* CountsCalculator::workBuffer = nullptr;
    float* CountsCalculator::downsampleBuffer = nullptr;

    bool CountsCalculator::AllocBuffers() {
      if (workBuffer == nullptr) {
        workBuffer = static_cast<float*>(pvPortMalloc(sizeof(float) * extendedLength));
      }
      if (downsampleBuffer == nullptr) {
        downsampleBuffer = static_cast<float*>(pvPortMalloc(sizeof(float) * downsampledLength));
      }
      return workBuffer != nullptr && downsampleBuffer != nullptr;
    }

    void CountsCalculator::FreeBuffers() {
      if (workBuffer != nullptr) {
        vPortFree(workBuffer);
        workBuffer = nullptr;
      }
      if (downsampleBuffer != nullptr) {
        vPortFree(downsampleBuffer);
        downsampleBuffer = nullptr;
      }
    }

    float CountsCalculator::ProcessAxis(const float* scaledData, size_t length) {
      if (length < 2) {
        return 0.0f;
      }

      // Step 1: Up-sample 10Hz raw data to 30Hz using linear interpolation
      const size_t resampledLength = length * resampleFactor;
      if (resampledLength <= filtfiltEdge || workBuffer == nullptr) {
        return 0.0f;
      }
      float* resampled = workBuffer + filtfiltEdge; // leave space for leading extension

      size_t outIdx = 0;
      for (size_t i = 0; i + 1 < length; i++) {
        float v0 = scaledData[i];
        float v1 = scaledData[i + 1];
        float step = (v1 - v0) * (1.0f / static_cast<float>(resampleFactor));

        resampled[outIdx++] = v0;
        resampled[outIdx++] = v0 + step;
        resampled[outIdx++] = v0 + 2.0f * step;
      }

      // Pad tail with the last sample to reach the expected length
      float lastValue = scaledData[length - 1];
      while (outIdx < resampledLength) {
        resampled[outIdx++] = lastValue;
      }

      // Step 2: Create edge extensions (odd reflection) around resampled data
      for (int i = 0; i < filtfiltEdge; i++) {
        workBuffer[i] = 2.0f * resampled[0] - resampled[filtfiltEdge - 1 - i];
      }
      for (int i = 0; i < filtfiltEdge; i++) {
        workBuffer[filtfiltEdge + resampledLength + i] =
          2.0f * resampled[resampledLength - 1] - resampled[resampledLength - 2 - i];
      }

      // Apply 8th-order filtfilt (forward + reverse using the same forward routine)
      filterAB2Forward(workBuffer, resampledLength + 2 * filtfiltEdge);
      std::reverse(workBuffer, workBuffer + resampledLength + 2 * filtfiltEdge);
      filterAB2Forward(workBuffer, resampledLength + 2 * filtfiltEdge);
      std::reverse(workBuffer, workBuffer + resampledLength + 2 * filtfiltEdge);

      // Apply 20th order filter in-place on the resampled data
      filterAB(workBuffer + filtfiltEdge, resampledLength);

      // Apply gain to match reference implementation
      for (size_t i = 0; i < resampledLength; i++) {
        workBuffer[filtfiltEdge + i] *= brondGain;
      }

      // Remove any residual DC/gravity bias to avoid inflating counts
      float mean = 0.0f;
      for (size_t i = 0; i < resampledLength; i++) {
        mean += workBuffer[filtfiltEdge + i];
      }
      mean /= static_cast<float>(resampledLength);
      for (size_t i = 0; i < resampledLength; i++) {
        workBuffer[filtfiltEdge + i] -= mean;
      }
      
      // Downsample by 3 back to 10Hz, clip, deadband, quantize into downsampleBuffer
      size_t j = 0;
      for (size_t i = 0; i < resampledLength; i += resampleFactor) {
        float d = workBuffer[filtfiltEdge + i];
        
        // Clip to peak threshold
        if (d < -peakThreshold) d = -peakThreshold;
        else if (d > peakThreshold) d = peakThreshold;
        
        // Apply deadband and quantize
        float absD = std::fabs(d);
        if (absD < deadband) {
          downsampleBuffer[j] = 0.0f;
        } else {
          downsampleBuffer[j] = std::floor(absD / adcResolution);
        }
        j++;
      }
      
      // Calculate running sum
      return runsum(downsampleBuffer, j, integN, 0.0f);
    }

    float CountsCalculator::Calculate(const int16_t* rawX, const int16_t* rawY, const int16_t* rawZ, size_t length) {
      if (length == 0 || length > maxInputLength) {
        return 0.0f;
      }
      if (!AllocBuffers()) {
        return 0.0f;
      }

      // Map raw milli-g values to g (1g = 1024) before processing
      // Reuse downsampleBuffer for the temporary mapped data to avoid extra RAM.
      // Remove DC (gravity) per-axis before filtering to avoid clipping.
      float mean = 0.0f;

      mean = 0.0f;
      for (size_t i = 0; i < length; i++) {
        downsampleBuffer[i] = static_cast<float>(rawX[i]) * scale;
        mean += downsampleBuffer[i];
      }
      mean /= static_cast<float>(length);
      for (size_t i = 0; i < length; i++) {
        downsampleBuffer[i] -= mean;
      }
      float x = ProcessAxis(downsampleBuffer, length);

      mean = 0.0f;
      for (size_t i = 0; i < length; i++) {
        downsampleBuffer[i] = static_cast<float>(rawY[i]) * scale;
        mean += downsampleBuffer[i];
      }
      mean /= static_cast<float>(length);
      for (size_t i = 0; i < length; i++) {
        downsampleBuffer[i] -= mean;
      }
      float y = ProcessAxis(downsampleBuffer, length);

      mean = 0.0f;
      for (size_t i = 0; i < length; i++) {
        downsampleBuffer[i] = static_cast<float>(rawZ[i]) * scale;
        mean += downsampleBuffer[i];
      }
      mean /= static_cast<float>(length);
      for (size_t i = 0; i < length; i++) {
        downsampleBuffer[i] -= mean;
      }
      float z = ProcessAxis(downsampleBuffer, length);

      float result = std::sqrt(x * x + y * y + z * z);
      FreeBuffers();
      return result;
    }
  } // namespace Utility
} // namespace Pinetime
