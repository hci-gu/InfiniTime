#pragma once

#include <array>
#include <cstdint>

#include <FreeRTOS.h>

#include "drivers/Bma421.h"
#include "components/ble/MotionService.h"
#include "components/datetime/DateTimeController.h"
#include "components/fs/FS.h"
#include "utility/CircularBuffer.h"
#include "utility/Counts.h"

namespace Pinetime {
  namespace Controllers {
    // RAII wrapper for LittleFS file handles to ensure cleanup on all paths
    class ScopedFile {
    public:
      ScopedFile(FS* fs) : fs(fs), isOpen(false) {}
      ~ScopedFile() { Close(); }

      // Non-copyable
      ScopedFile(const ScopedFile&) = delete;
      ScopedFile& operator=(const ScopedFile&) = delete;

      bool Open(const char* path, int flags) {
        Close();
        if (fs != nullptr && fs->FileOpen(&file, path, flags) == LFS_ERR_OK) {
          isOpen = true;
          return true;
        }
        return false;
      }

      void Close() {
        if (isOpen && fs != nullptr) {
          fs->FileClose(&file);
          isOpen = false;
        }
      }

      bool IsOpen() const { return isOpen; }
      lfs_file_t* Get() { return &file; }

      int Read(void* buffer, size_t size) {
        return isOpen ? fs->FileRead(&file, static_cast<uint8_t*>(buffer), size) : -1;
      }

      int Write(const void* buffer, size_t size) {
        return isOpen ? fs->FileWrite(&file, static_cast<const uint8_t*>(buffer), size) : -1;
      }

      int Seek(lfs_soff_t offset) {
        return isOpen ? fs->FileSeek(&file, offset) : -1;
      }

    private:
      FS* fs;
      lfs_file_t file;
      bool isOpen;
    };

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

      void AddHeartRateSample(TickType_t timestamp, uint16_t heartRate);

      DeviceTypes DeviceType() const {
        return deviceType;
      }

      void Init(Pinetime::Drivers::Bma421::DeviceTypes types, Pinetime::Controllers::FS& fs, Pinetime::Controllers::DateTime& dateTimeController);

      void OnStorageWake();
      void OnStorageSleep();

      void SetService(Pinetime::Controllers::MotionService* service) {
        this->service = service;
      }

      Pinetime::Controllers::MotionService* GetService() const {
        return service;
      }

      size_t LoggedMinuteCount() const {
        return minuteAverageCount;
      }

      float LoggedMinutesAverage() const;
      int32_t LoggedMinutesHeartRateAverage() const;
      bool HasLoggedHeartRateAverage() const {
        return minuteHeartRateSampleCount > 0;
      }

      void ClearMinuteAverageLog();

      // Public method to flush buffered data to disk storage
      // Called by SystemTask during periodic wake-ups to ensure data is persisted
      void FlushToStorage();

      // BLE data access methods
      struct MinuteEntryData {
        float counts;
        int16_t heartRate;
        uint32_t timestamp;
      };
      size_t GetStoredEntryCount() const;
      bool ReadStoredEntry(size_t index, MinuteEntryData& entry);
      void FlushAndClearStoredData();

    private:
      Utility::CircularBuffer<uint32_t, stepHistorySize> nbSteps = {0};
      uint32_t currentTripSteps = 0;

      void SetSteps(Days day, uint32_t steps) {
        nbSteps[static_cast<std::underlying_type_t<Days>>(day)] = steps;
      }

      TickType_t lastTime = 0;
      TickType_t time = 0;

      struct HeartRateSample {
        TickType_t timestamp = 0;
        uint16_t value = 0;
      };

      void AddAccelerationSample(TickType_t timestamp, int16_t x, int16_t y, int16_t z);
      void PruneOldHeartRateSamplesLocked(TickType_t currentTimestamp);

      // Raw X/Y/Z samples for Counts calculation (circular buffer)
      // Stored as int16_t in binary milli-g format (1g = 1024)
      static constexpr size_t accelSamplesWindow = 600; // 10Hz * 60s
      std::array<int16_t, accelSamplesWindow> rawXSamples = {};
      std::array<int16_t, accelSamplesWindow> rawYSamples = {};
      std::array<int16_t, accelSamplesWindow> rawZSamples = {};
      size_t rawSampleHead = 0;
      size_t rawSampleCount = 0;

      // Heart-rate readings arrive less frequently than acceleration (typically 1Hz).
      // 60 samples covers a full minute at 1Hz.
      static constexpr size_t heartRateSamplesWindow = 60;
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

      // Maximum entries stored on disk (24 hours at 1 per minute)
      static constexpr size_t maxDiskLogEntries = 1440;
      // In-memory buffer to batch writes (10 minutes - balances memory usage with data safety)
      // Timer flushes every 5 minutes, so 10 minutes provides a safety margin
      static constexpr size_t inMemoryBufferSize = 10;
      static constexpr TickType_t minuteDurationTicks = configTICK_RATE_HZ * 60;
      static constexpr uint32_t minuteAverageLogVersion = 7;  // v7: Counts algorithm/rate update, clear old data
      static constexpr const char minuteAverageDirectory[] = "/.system";
      static constexpr const char minuteAverageFile[] = "/.system/accel_avg.dat";

      struct MinuteEntry {
        float counts = 0.0f;
        int16_t heartRate = 0;
        uint32_t timestamp = 0;
      };

      Pinetime::Controllers::FS* fs = nullptr;
      Pinetime::Controllers::DateTime* dateTimeController = nullptr;

      // Small circular buffer for recent entries not yet flushed to disk
      std::array<MinuteEntry, inMemoryBufferSize> inMemoryBuffer = {};
      size_t inMemoryBufferCount = 0;

      // Running totals (disk + in-memory combined) for computing averages
      size_t diskEntryCount = 0;        // Number of entries stored on disk
      size_t minuteAverageCount = 0;    // Total entries (disk + in-memory)
      double minuteAverageTotal = 0.0;  // Sum of all counts values
      int64_t minuteHeartRateTotal = 0; // Sum of all heart rate values
      size_t minuteHeartRateSampleCount = 0; // Count of non-zero heart rate entries
      TickType_t lastLoggedMinuteTick = 0;
      uint32_t baseUnixTimestamp = 0;      // Unix timestamp when minute logging started
      TickType_t baseMinuteTick = 0;       // Tick count when minute logging started
      bool storageAccessible = true;

      void LoadMinuteAverageLog();
      void FlushBufferToDisk();
      void EnsureLogDirectory();
      void AppendMinuteAverage(float counts, int32_t heartRateAverage, uint32_t timestamp);
      void MaybeStoreMinuteAverage(TickType_t timestamp);
      float CalculateCountsFromRawSamples();
      int32_t AverageHeartRateLastMinuteInternal(TickType_t currentTimestamp);
      void TruncateDiskLogIfNeeded();
    };
  }
}
