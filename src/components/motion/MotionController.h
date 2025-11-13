#pragma once

#include <array>
#include <cstdint>
#include <chrono>

#include <FreeRTOS.h>
#include <semphr.h>

#include "drivers/Bma421.h"
#include "components/fs/FS.h"
#include "utility/CircularBuffer.h"
#include "utility/Span.h"

namespace Pinetime {
  namespace Controllers {
    class DateTime;
    class MotionService;

    class MotionController {
    public:
      enum class DeviceTypes {
        Unknown,
        BMA421,
        BMA425,
      };

      enum class Days : uint8_t {
        Today = 0,
        Yesterday,
      };

      static constexpr size_t stepHistorySize = 2; // Store this many day's step counter

      void AdvanceDay();

      void Update(int16_t x, int16_t y, int16_t z, uint32_t nbSteps);

      int16_t X() const {
        return xHistory[0];
      }

      int16_t Y() const {
        return yHistory[0];
      }

      int16_t Z() const {
        return zHistory[0];
      }

      uint32_t NbSteps(Days day = Days::Today) const {
        return nbSteps[static_cast<std::underlying_type_t<Days>>(day)];
      }

      void ResetTrip() {
        currentTripSteps = 0;
      }

      uint32_t GetTripSteps() const {
        return currentTripSteps;
      }

      bool ShouldRaiseWake() const;
      bool ShouldLowerSleep() const;

      int32_t CurrentShakeSpeed() const {
        return accumulatedSpeed;
      }

      int32_t AverageAccelerationLastMinute();
      void AddHeartRateSample(TickType_t timestamp, uint16_t heartRate);

      DeviceTypes DeviceType() const {
        return deviceType;
      }

      void Init(Pinetime::Drivers::Bma421::DeviceTypes types, Pinetime::Controllers::FS& fs);

      void OnStorageWake();
      void OnStorageSleep();

      void SetService(Pinetime::Controllers::MotionService* service) {
        this->service = service;
      }

      Pinetime::Controllers::MotionService* GetService() const {
        return service;
      }

      size_t LoggedMinuteCount() const;

      int32_t LoggedMinutesAverage() const;
      int32_t LoggedMinutesHeartRateAverage() const;
      bool HasLoggedHeartRateAverage() const;

      void ClearMinuteAverageLog();

      struct MinuteSample {
        uint32_t timestamp = 0;
        int16_t accelerationAverage = 0;
        int16_t heartRateAverage = 0;
        uint8_t flags = 0;
      };

      static constexpr size_t MinuteLogCapacity = 1440;

      enum MinuteSampleFlags : uint8_t {
        SampleFlagNone = 0,
        SampleFlagMissingHeartRate = 1u << 0,
      };

      struct MinuteLogRange {
        uint32_t oldestEpoch = 0;
        uint32_t newestEpoch = 0;
        size_t count = 0;
      };

      MinuteLogRange OldestNewest() const;
      size_t CopyMinutes(uint32_t startEpoch, size_t maxSamples, gsl::span<MinuteSample> out) const;
      void MarkMinutesTransferred(uint32_t upToEpoch);

      void SetDateTimeController(Pinetime::Controllers::DateTime* controller) {
        dateTimeController = controller;
      }

    private:
      Utility::CircularBuffer<uint32_t, stepHistorySize> nbSteps = {0};
      uint32_t currentTripSteps = 0;

      void SetSteps(Days day, uint32_t steps) {
        nbSteps[static_cast<std::underlying_type_t<Days>>(day)] = steps;
      }

      TickType_t lastTime = 0;
      TickType_t time = 0;

      struct AccelSample {
        TickType_t timestamp = 0;
        int32_t magnitude = 0;
      };

      struct HeartRateSample {
        TickType_t timestamp = 0;
        uint16_t value = 0;
      };

      void AddAccelerationSample(TickType_t timestamp, int32_t magnitude);
      void PruneOldAccelerationSamples(TickType_t currentTimestamp);
      void PruneOldHeartRateSamplesLocked(TickType_t currentTimestamp);

      static constexpr size_t accelSamplesWindow = 600; // 10Hz * 60s
      // Heart-rate readings arrive far less frequently than acceleration (typically 1Hz).
      // Keeping a full 60s history for up to ~4 samples per second keeps memory usage low
      // enough to avoid boot-time allocation failures while still covering the full minute.
      static constexpr size_t heartRateSamplesWindow = 240;
      std::array<AccelSample, accelSamplesWindow> accelSamples = {};
      size_t accelSampleHead = 0;
      size_t accelSampleTail = 0;
      size_t accelSampleCount = 0;
      int64_t accelSampleTotal = 0;

      std::array<HeartRateSample, heartRateSamplesWindow> heartRateSamples = {};
      size_t heartRateSampleHead = 0;
      size_t heartRateSampleTail = 0;
      size_t heartRateSampleCount = 0;
      int64_t heartRateSampleTotal = 0;

      struct AccelStats {
        static constexpr uint8_t numHistory = 2;

        int16_t xMean = 0;
        int16_t yMean = 0;
        int16_t zMean = 0;
        int16_t prevXMean = 0;
        int16_t prevYMean = 0;
        int16_t prevZMean = 0;

        uint32_t xVariance = 0;
        uint32_t yVariance = 0;
        uint32_t zVariance = 0;
      };

      AccelStats GetAccelStats() const;

      AccelStats stats = {};

      static constexpr uint8_t histSize = 8;
      Utility::CircularBuffer<int16_t, histSize> xHistory = {};
      Utility::CircularBuffer<int16_t, histSize> yHistory = {};
      Utility::CircularBuffer<int16_t, histSize> zHistory = {};
      int32_t accumulatedSpeed = 0;

      DeviceTypes deviceType = DeviceTypes::Unknown;
      Pinetime::Controllers::MotionService* service = nullptr;

      static constexpr size_t minuteAverageLogSize = MinuteLogCapacity;
      static constexpr TickType_t minuteDurationTicks = configTICK_RATE_HZ * 60;
      static constexpr uint32_t minuteAverageLogVersion = 3;
      static constexpr const char minuteAverageDirectory[] = "/.system";
      static constexpr const char minuteAverageFile[] = "/.system/accel_avg.dat";

      Pinetime::Controllers::FS* fs = nullptr;

      std::array<int32_t, minuteAverageLogSize> minuteAccelerationAverages = {};
      std::array<int16_t, minuteAverageLogSize> minuteHeartRateAverages = {};
      size_t minuteAverageStart = 0;
      size_t minuteAverageCount = 0;
      int64_t minuteAverageTotal = 0;
      int64_t minuteHeartRateTotal = 0;
      size_t minuteHeartRateSampleCount = 0;
      TickType_t lastLoggedMinuteTick = 0;
      bool minuteAverageDirty = false;
      bool storageAccessible = true;
      std::array<uint32_t, minuteAverageLogSize> minuteTimestamps = {};
      std::array<uint8_t, minuteAverageLogSize> minuteFlags = {};

      mutable SemaphoreHandle_t minuteLogMutex = nullptr;
      mutable StaticSemaphore_t minuteLogMutexBuffer = {};

      Pinetime::Controllers::DateTime* dateTimeController = nullptr;

      class MinuteLogLock {
      public:
        explicit MinuteLogLock(const MotionController& controller);
        ~MinuteLogLock();

      private:
        const MotionController& controller;
      };

      void LockMinuteLog() const;
      void UnlockMinuteLog() const;
      uint32_t CurrentEpochSeconds() const;
      uint32_t NormalizeEpochToMinute(uint32_t epochSeconds) const;
      void InitializeMinuteLogMutex();

      void LoadMinuteAverageLog();
      void SaveMinuteAverageLog();
      void EnsureLogDirectory();
      void AppendMinuteAverage(int32_t accelerationAverage,
                              int32_t heartRateAverage,
                              uint32_t epochSeconds,
                              uint8_t flags);
      void MaybeStoreMinuteAverage(TickType_t timestamp);
      int32_t AverageAccelerationLastMinuteInternal(TickType_t currentTimestamp);
      int32_t AverageHeartRateLastMinuteInternal(TickType_t currentTimestamp, bool* hasSample = nullptr);
      void MaybePersistMinuteAverageLog();
    };
  }
}
