#include "components/motion/MotionController.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <task.h>

#include "utility/Math.h"

using namespace Pinetime::Controllers;

namespace {
  // Packed structs for consistent binary layout on disk
  // These MUST be packed to avoid platform-dependent alignment padding
  struct __attribute__((packed)) DiskHeader {
    uint32_t version;
    uint32_t count;
    double totalCounts;
    int64_t totalHeartRate;
    uint32_t heartRateSampleCount;
  };

  struct __attribute__((packed)) DiskEntry {
    float counts;
    int16_t heartRate;
    uint32_t timestamp;
  };

  constexpr inline int32_t Clamp(int32_t val, int32_t min, int32_t max) {
    return val < min ? min : (val > max ? max : val);
  }

  // only returns meaningful values if inputs are acceleration due to gravity
  int16_t DegreesRolled(int16_t y, int16_t z, int16_t prevY, int16_t prevZ) {
    int16_t prevYAngle = Pinetime::Utility::Asin(Clamp(prevY * 32, -32767, 32767));
    int16_t yAngle = Pinetime::Utility::Asin(Clamp(y * 32, -32767, 32767));

    if (z < 0 && prevZ < 0) {
      return yAngle - prevYAngle;
    }
    if (prevZ < 0) {
      if (y < 0) {
        return -prevYAngle - yAngle - 180;
      }
      return -prevYAngle - yAngle + 180;
    }
    if (z < 0) {
      if (y < 0) {
        return prevYAngle + yAngle + 180;
      }
      return prevYAngle + yAngle - 180;
    }
    return prevYAngle - yAngle;
  }
}

void MotionController::AdvanceDay() {
  --nbSteps; // Higher index = further in the past
  SetSteps(Days::Today, 0);
  if (service != nullptr) {
    service->OnNewStepCountValue(NbSteps(Days::Today));
  }
}

void MotionController::Update(int16_t x, int16_t y, int16_t z, uint32_t nbSteps) {
  uint32_t oldSteps = NbSteps(Days::Today);
  if (oldSteps != nbSteps && service != nullptr) {
    service->OnNewStepCountValue(nbSteps);
  }

  if (service != nullptr && (xHistory[0] != x || yHistory[0] != y || zHistory[0] != z)) {
    service->OnNewMotionValues(x, y, z);
  }

  lastTime = time;
  time = xTaskGetTickCount();

  xHistory++;
  xHistory[0] = x;
  yHistory++;
  yHistory[0] = y;
  zHistory++;
  zHistory[0] = z;

  // Update accumulated speed
  // Currently polling at 10Hz, if this ever goes faster scalar and EMA might need adjusting
  int32_t speed = std::abs(zHistory[0] - zHistory[histSize - 1] + ((yHistory[0] - yHistory[histSize - 1]) / 2) +
                           ((xHistory[0] - xHistory[histSize - 1]) / 4)) *
                  100 / (time - lastTime);
  // integer version of (.2 * speed) + ((1 - .2) * accumulatedSpeed);
  accumulatedSpeed = speed / 5 + accumulatedSpeed * 4 / 5;

  stats = GetAccelStats();

  int64_t magnitudeSquared = static_cast<int64_t>(x) * x + static_cast<int64_t>(y) * y + static_cast<int64_t>(z) * z;
  int32_t magnitude = static_cast<int32_t>(std::sqrt(static_cast<double>(magnitudeSquared)));
  (void)magnitude; // No longer used for averaging, kept for potential future use
  AddAccelerationSample(time, x, y, z);

  int32_t deltaSteps = nbSteps - oldSteps;
  if (deltaSteps > 0) {
    currentTripSteps += deltaSteps;
  }
  SetSteps(Days::Today, nbSteps);
}

float MotionController::LoggedMinutesAverage() const {
  if (minuteAverageCount == 0) {
    return 0.0f;
  }

  return static_cast<float>(minuteAverageTotal / static_cast<double>(minuteAverageCount));
}

int32_t MotionController::LoggedMinutesHeartRateAverage() const {
  if (minuteHeartRateSampleCount == 0) {
    return 0;
  }

  return static_cast<int32_t>(minuteHeartRateTotal / static_cast<int64_t>(minuteHeartRateSampleCount));
}

void MotionController::ClearMinuteAverageLog() {
  // Clear in-memory buffer
  inMemoryBufferCount = 0;
  std::fill(inMemoryBuffer.begin(), inMemoryBuffer.end(), MinuteEntry{});

  // Clear running totals
  diskEntryCount = 0;
  minuteAverageCount = 0;
  minuteAverageTotal = 0.0;
  minuteHeartRateTotal = 0;
  minuteHeartRateSampleCount = 0;

  // Reset timestamp tracking so next entry will re-establish the base
  baseUnixTimestamp = 0;
  baseMinuteTick = 0;

  // Delete the disk file
  if (fs != nullptr && storageAccessible) {
    fs->FileDelete(minuteAverageFile);
  }
}

MotionController::AccelStats MotionController::GetAccelStats() const {
  AccelStats stats;

  for (uint8_t i = 0; i < AccelStats::numHistory; i++) {
    stats.xMean += xHistory[histSize - i];
    stats.yMean += yHistory[histSize - i];
    stats.zMean += zHistory[histSize - i];
    stats.prevXMean += xHistory[1 + i];
    stats.prevYMean += yHistory[1 + i];
    stats.prevZMean += zHistory[1 + i];
  }
  stats.xMean /= AccelStats::numHistory;
  stats.yMean /= AccelStats::numHistory;
  stats.zMean /= AccelStats::numHistory;
  stats.prevXMean /= AccelStats::numHistory;
  stats.prevYMean /= AccelStats::numHistory;
  stats.prevZMean /= AccelStats::numHistory;

  for (uint8_t i = 0; i < AccelStats::numHistory; i++) {
    stats.xVariance += (xHistory[histSize - i] - stats.xMean) * (xHistory[histSize - i] - stats.xMean);
    stats.yVariance += (yHistory[histSize - i] - stats.yMean) * (yHistory[histSize - i] - stats.yMean);
    stats.zVariance += (zHistory[histSize - i] - stats.zMean) * (zHistory[histSize - i] - stats.zMean);
  }
  stats.xVariance /= AccelStats::numHistory;
  stats.yVariance /= AccelStats::numHistory;
  stats.zVariance /= AccelStats::numHistory;

  return stats;
}

bool MotionController::ShouldRaiseWake() const {
  constexpr uint32_t varianceThresh = 56 * 56;
  constexpr int16_t xThresh = 384;
  constexpr int16_t yThresh = -64;
  constexpr int16_t rollDegreesThresh = -45;

  if (std::abs(stats.xMean) > xThresh) {
    return false;
  }

  // if the variance is below the threshold, the accelerometer values can be considered to be from acceleration due to gravity
  if (stats.yVariance > varianceThresh || (stats.yMean < -724 && stats.zVariance > varianceThresh) || stats.yMean > yThresh) {
    return false;
  }

  return DegreesRolled(stats.yMean, stats.zMean, stats.prevYMean, stats.prevZMean) < rollDegreesThresh;
}

bool MotionController::ShouldLowerSleep() const {
  if ((stats.xMean > 887 && DegreesRolled(stats.xMean, stats.zMean, stats.prevXMean, stats.prevZMean) > 30) ||
      (stats.xMean < -887 && DegreesRolled(stats.xMean, stats.zMean, stats.prevXMean, stats.prevZMean) < -30)) {
    return true;
  }

  if (stats.yMean < 724 || DegreesRolled(stats.yMean, stats.zMean, stats.prevYMean, stats.prevZMean) < 30) {
    return false;
  }

  for (uint8_t i = AccelStats::numHistory + 1; i < yHistory.Size(); i++) {
    if (yHistory[i] < 265) {
      return false;
    }
  }

  return true;
}

void MotionController::Init(Pinetime::Drivers::Bma421::DeviceTypes types, Pinetime::Controllers::FS& fsController, Pinetime::Controllers::DateTime& dateTime) {
  switch (types) {
    case Drivers::Bma421::DeviceTypes::BMA421:
      this->deviceType = DeviceTypes::BMA421;
      break;
    case Drivers::Bma421::DeviceTypes::BMA425:
      this->deviceType = DeviceTypes::BMA425;
      break;
    default:
      this->deviceType = DeviceTypes::Unknown;
      break;
  }

  fs = &fsController;
  dateTimeController = &dateTime;
  storageAccessible = true;
  LoadMinuteAverageLog();
  lastLoggedMinuteTick = xTaskGetTickCount();
}

void MotionController::OnStorageWake() {
  storageAccessible = true;
  // Flush any pending in-memory entries to disk
  if (inMemoryBufferCount > 0) {
    FlushBufferToDisk();
  }
}

void MotionController::OnStorageSleep() {
  storageAccessible = false;
}

void MotionController::AddAccelerationSample(TickType_t timestamp, int16_t x, int16_t y, int16_t z) {
  // Store raw X/Y/Z samples for Counts calculation (circular buffer)
  rawXSamples[rawSampleHead] = x;
  rawYSamples[rawSampleHead] = y;
  rawZSamples[rawSampleHead] = z;
  rawSampleHead = (rawSampleHead + 1) % accelSamplesWindow;
  if (rawSampleCount < accelSamplesWindow) {
    rawSampleCount++;
  }

  MaybeStoreMinuteAverage(timestamp);
}

void MotionController::AddHeartRateSample(TickType_t timestamp, uint16_t heartRate) {
  if (heartRate == 0) {
    return;
  }

  taskENTER_CRITICAL();

  if (heartRateSampleCount == heartRateSamplesWindow) {
    heartRateSampleTotal -= heartRateSamples[heartRateSampleTail].value;
    heartRateSampleTail = (heartRateSampleTail + 1) % heartRateSamplesWindow;
    heartRateSampleCount--;
  }

  heartRateSamples[heartRateSampleHead].timestamp = timestamp;
  heartRateSamples[heartRateSampleHead].value = heartRate;
  heartRateSampleHead = (heartRateSampleHead + 1) % heartRateSamplesWindow;

  heartRateSampleCount++;
  heartRateSampleTotal += heartRate;

  PruneOldHeartRateSamplesLocked(timestamp);

  taskEXIT_CRITICAL();
}

void MotionController::PruneOldHeartRateSamplesLocked(TickType_t currentTimestamp) {
  constexpr TickType_t window = configTICK_RATE_HZ * 60;

  while (heartRateSampleCount > 0) {
    const auto& oldest = heartRateSamples[heartRateSampleTail];
    TickType_t age = currentTimestamp - oldest.timestamp;
    if (age <= window) {
      break;
    }

    heartRateSampleTotal -= oldest.value;
    heartRateSampleTail = (heartRateSampleTail + 1) % heartRateSamplesWindow;
    heartRateSampleCount--;
  }
}

int32_t MotionController::AverageHeartRateLastMinuteInternal(TickType_t currentTimestamp) {
  taskENTER_CRITICAL();
  PruneOldHeartRateSamplesLocked(currentTimestamp);

  if (heartRateSampleCount == 0) {
    taskEXIT_CRITICAL();
    return 0;
  }

  int64_t total = heartRateSampleTotal;
  int64_t count = heartRateSampleCount;
  taskEXIT_CRITICAL();

  return static_cast<int32_t>(total / count);
}

float MotionController::CalculateCountsFromRawSamples() {
  if (rawSampleCount == 0) {
    return 0.0f;
  }

  // The raw samples are stored in circular buffers. For Counts calculation,
  // we need them linearized (oldest to newest).
  // rawSampleHead points to where the next sample will be written.
  // If buffer is full, oldest is at rawSampleHead; if not full, oldest is at 0.
  size_t startIdx = (rawSampleCount == accelSamplesWindow) ? rawSampleHead : 0;

  // If data is already linearized (starts at 0), we can use it directly
  // Otherwise, we need to rotate in-place using std::rotate
  if (startIdx != 0) {
    std::rotate(rawXSamples.begin(), rawXSamples.begin() + startIdx, rawXSamples.begin() + rawSampleCount);
    std::rotate(rawYSamples.begin(), rawYSamples.begin() + startIdx, rawYSamples.begin() + rawSampleCount);
    std::rotate(rawZSamples.begin(), rawZSamples.begin() + startIdx, rawZSamples.begin() + rawSampleCount);
  }

  // Calculate counts from linearized int16_t data
  // CountsCalculator handles conversion to float internally
  float counts = Utility::CountsCalculator::Calculate(
    rawXSamples.data(),
    rawYSamples.data(),
    rawZSamples.data(),
    rawSampleCount
  );

  // Reset raw sample buffers for next minute
  rawSampleHead = 0;
  rawSampleCount = 0;

  return counts;
}

void MotionController::MaybeStoreMinuteAverage(TickType_t timestamp) {
  if (fs == nullptr) {
    return;
  }

  if (lastLoggedMinuteTick == 0) {
    lastLoggedMinuteTick = timestamp;
    return;
  }

  if (timestamp - lastLoggedMinuteTick < minuteDurationTicks) {
    return;
  }

  float counts = CalculateCountsFromRawSamples();
  int32_t heartRateAverage = AverageHeartRateLastMinuteInternal(timestamp);

  if (counts == 0.0f && rawSampleCount == 0) {
    lastLoggedMinuteTick = timestamp;
    return;
  }

  // Calculate Unix timestamp based on relative tick offset from base
  // This prevents duplicate timestamps when multiple entries are stored in quick succession
  uint32_t unixTimestamp = 0;
  if (dateTimeController != nullptr) {
    if (baseUnixTimestamp == 0) {
      // Initialize base timestamp on first entry
      auto now = dateTimeController->CurrentDateTime();
      baseUnixTimestamp = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count());
      baseMinuteTick = timestamp;
      unixTimestamp = baseUnixTimestamp;
    } else {
      // Calculate timestamp relative to base using tick count
      // Each minuteDurationTicks corresponds to 60 seconds
      TickType_t ticksSinceBase = timestamp - baseMinuteTick;
      uint32_t minutesSinceBase = ticksSinceBase / minuteDurationTicks;
      unixTimestamp = baseUnixTimestamp + (minutesSinceBase * 60);
    }
  }

  AppendMinuteAverage(counts, heartRateAverage, unixTimestamp);
  lastLoggedMinuteTick = timestamp;
}

void MotionController::AppendMinuteAverage(float counts, int32_t heartRateAverage, uint32_t timestamp) {
  const int32_t clampedHeartRate = std::clamp<int32_t>(
    heartRateAverage, 0, static_cast<int32_t>(std::numeric_limits<int16_t>::max()));
  const int16_t storedHeartRate = static_cast<int16_t>(clampedHeartRate);

  // If buffer is full, try to flush first
  if (inMemoryBufferCount >= inMemoryBufferSize) {
    FlushBufferToDisk();

    // If flush failed (buffer still full), discard oldest entry to make room for newest
    // This prevents buffer overflow - data loss is preferable to crash
    if (inMemoryBufferCount >= inMemoryBufferSize) {
      // Subtract oldest entry from running totals
      minuteAverageTotal -= inMemoryBuffer[0].counts;
      minuteAverageCount--;
      if (inMemoryBuffer[0].heartRate > 0) {
        minuteHeartRateTotal -= inMemoryBuffer[0].heartRate;
        minuteHeartRateSampleCount--;
      }

      // Shift entries left to discard oldest
      for (size_t i = 1; i < inMemoryBufferSize; ++i) {
        inMemoryBuffer[i - 1] = inMemoryBuffer[i];
      }
      inMemoryBufferCount--;
    }
  }

  // Add to in-memory buffer (now guaranteed to have space)
  inMemoryBuffer[inMemoryBufferCount].counts = counts;
  inMemoryBuffer[inMemoryBufferCount].heartRate = storedHeartRate;
  inMemoryBuffer[inMemoryBufferCount].timestamp = timestamp;
  inMemoryBufferCount++;

  // Update running totals
  minuteAverageTotal += counts;
  minuteAverageCount++;
  if (storedHeartRate > 0) {
    minuteHeartRateTotal += storedHeartRate;
    minuteHeartRateSampleCount++;
  }
}

void MotionController::FlushBufferToDisk() {
  if (fs == nullptr || !storageAccessible || inMemoryBufferCount == 0) {
    return;
  }

  EnsureLogDirectory();

  // Check if we need to truncate (handle wrap-around at maxDiskLogEntries)
  TruncateDiskLogIfNeeded();

  ScopedFile file(fs);

  // If file doesn't exist or is empty, create with header
  if (!file.Open(minuteAverageFile, LFS_O_RDONLY)) {
    // File doesn't exist, create it with header
    if (!file.Open(minuteAverageFile, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC)) {
      return;
    }

    DiskHeader header {
      minuteAverageLogVersion,
      static_cast<uint32_t>(inMemoryBufferCount),
      minuteAverageTotal,
      minuteHeartRateTotal,
      static_cast<uint32_t>(minuteHeartRateSampleCount)
    };

    if (file.Write(&header, sizeof(header)) != sizeof(header)) {
      return; // ScopedFile destructor will close
    }

    // Write all buffered entries

    for (size_t i = 0; i < inMemoryBufferCount; ++i) {
      DiskEntry entry {inMemoryBuffer[i].counts, inMemoryBuffer[i].heartRate, inMemoryBuffer[i].timestamp};
      if (file.Write(&entry, sizeof(entry)) != sizeof(entry)) {
        return; // ScopedFile destructor will close
      }
    }

    diskEntryCount = inMemoryBufferCount;
    inMemoryBufferCount = 0;
    return;
  }

  file.Close();

  // File exists, read current header, update count and totals, then append entries
  if (!file.Open(minuteAverageFile, LFS_O_RDWR)) {
    return;
  }

  DiskHeader header {};

  if (file.Read(&header, sizeof(header)) != sizeof(header)) {
    return; // ScopedFile destructor will close
  }

  // Update header with new totals
  header.count = static_cast<uint32_t>(diskEntryCount + inMemoryBufferCount);
  header.totalCounts = minuteAverageTotal;
  header.totalHeartRate = minuteHeartRateTotal;
  header.heartRateSampleCount = static_cast<uint32_t>(minuteHeartRateSampleCount);

  // Seek back to start and write updated header
  if (file.Seek(0) < 0) {
    return; // ScopedFile destructor will close
  }
  if (file.Write(&header, sizeof(header)) != sizeof(header)) {
    return; // ScopedFile destructor will close
  }

  // Seek to end to append new entries
  if (file.Seek(sizeof(DiskHeader) + diskEntryCount * sizeof(DiskEntry)) < 0) {
    return; // ScopedFile destructor will close
  }

  for (size_t i = 0; i < inMemoryBufferCount; ++i) {
    DiskEntry entry {inMemoryBuffer[i].counts, inMemoryBuffer[i].heartRate, inMemoryBuffer[i].timestamp};
    if (file.Write(&entry, sizeof(entry)) != sizeof(entry)) {
      return; // ScopedFile destructor will close
    }
  }

  diskEntryCount += inMemoryBufferCount;
  inMemoryBufferCount = 0;
}

void MotionController::TruncateDiskLogIfNeeded() {
  // If total entries would exceed max, we need to remove oldest entries from disk
  size_t totalAfterFlush = diskEntryCount + inMemoryBufferCount;
  if (totalAfterFlush <= maxDiskLogEntries) {
    return;
  }

  size_t entriesToRemove = totalAfterFlush - maxDiskLogEntries;

  if (fs == nullptr || !storageAccessible) {
    return;
  }

  ScopedFile file(fs);
  if (!file.Open(minuteAverageFile, LFS_O_RDONLY)) {
    return;
  }

  DiskHeader oldHeader {};

  if (file.Read(&oldHeader, sizeof(oldHeader)) != sizeof(oldHeader)) {
    return; // ScopedFile destructor will close
  }

  // Calculate new totals by reading and subtracting removed entries
  double removedCountsTotal = 0.0;
  int64_t removedHeartRateTotal = 0;
  size_t removedHeartRateSamples = 0;

  for (size_t i = 0; i < entriesToRemove && i < diskEntryCount; ++i) {
    DiskEntry entry {};
    if (file.Read(&entry, sizeof(entry)) != sizeof(entry)) {
      break;
    }
    removedCountsTotal += entry.counts;
    if (entry.heartRate > 0) {
      removedHeartRateTotal += entry.heartRate;
      removedHeartRateSamples++;
    }
  }

  // Stream remaining entries to a temp file
  size_t remainingEntries = diskEntryCount > entriesToRemove ? diskEntryCount - entriesToRemove : 0;

  static constexpr const char tempFile[] = "/.system/accel_avg.tmp";
  ScopedFile newFile(fs);

  if (!newFile.Open(tempFile, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC)) {
    return; // ScopedFile destructor will close file
  }

  // Calculate new totals for disk portion
  double newDiskCountsTotal = 0.0;
  int64_t newDiskHrTotal = 0;
  size_t newDiskHrCount = 0;

  // Write placeholder header (will update after streaming entries)
  DiskHeader newHeader {
    minuteAverageLogVersion,
    static_cast<uint32_t>(remainingEntries),
    0.0, 0, 0  // Will be updated
  };
  if (newFile.Write(&newHeader, sizeof(newHeader)) != sizeof(newHeader)) {
    return; // Both ScopedFile destructors will close files
  }

  // Stream entries from old file to new file
  for (size_t i = 0; i < remainingEntries; ++i) {
    DiskEntry entry {};
    if (file.Read(&entry, sizeof(entry)) != sizeof(entry)) {
      remainingEntries = i;
      break;
    }
    if (newFile.Write(&entry, sizeof(entry)) != sizeof(entry)) {
      return; // Both ScopedFile destructors will close files
    }
    newDiskCountsTotal += entry.counts;
    if (entry.heartRate > 0) {
      newDiskHrTotal += entry.heartRate;
      newDiskHrCount++;
    }
  }

  file.Close();

  // Update header with correct totals
  newHeader.count = static_cast<uint32_t>(remainingEntries);
  newHeader.totalCounts = newDiskCountsTotal;
  newHeader.totalHeartRate = newDiskHrTotal;
  newHeader.heartRateSampleCount = static_cast<uint32_t>(newDiskHrCount);

  if (newFile.Seek(0) < 0) {
    return; // ScopedFile destructor will close
  }
  if (newFile.Write(&newHeader, sizeof(newHeader)) != sizeof(newHeader)) {
    return; // ScopedFile destructor will close
  }
  newFile.Close();

  // Replace old file with new
  fs->FileDelete(minuteAverageFile);

  // Copy temp file to final location using ScopedFile for safety
  {
    ScopedFile srcFile(fs);
    ScopedFile dstFile(fs);

    if (srcFile.Open(tempFile, LFS_O_RDONLY)) {
      if (dstFile.Open(minuteAverageFile, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC)) {
        uint8_t buffer[64];
        int bytesRead;
        while ((bytesRead = srcFile.Read(buffer, sizeof(buffer))) > 0) {
          if (dstFile.Write(buffer, bytesRead) != bytesRead) {
            break; // Write failed, ScopedFile destructors will close both files
          }
        }
      }
    }
  } // ScopedFile destructors ensure both files are closed
  fs->FileDelete(tempFile);

  diskEntryCount = remainingEntries;

  // Recalculate running totals to include in-memory buffer
  minuteAverageTotal = newDiskCountsTotal;
  minuteHeartRateTotal = newDiskHrTotal;
  minuteHeartRateSampleCount = newDiskHrCount;
  minuteAverageCount = remainingEntries;

  for (size_t i = 0; i < inMemoryBufferCount; ++i) {
    minuteAverageTotal += inMemoryBuffer[i].counts;
    minuteAverageCount++;
    if (inMemoryBuffer[i].heartRate > 0) {
      minuteHeartRateTotal += inMemoryBuffer[i].heartRate;
      minuteHeartRateSampleCount++;
    }
  }
}

void MotionController::EnsureLogDirectory() {
  if (fs == nullptr) {
    return;
  }

  lfs_dir_t dir;
  if (fs->DirOpen(minuteAverageDirectory, &dir) == LFS_ERR_OK) {
    fs->DirClose(&dir);
    return;
  }

  fs->DirCreate(minuteAverageDirectory);
}

void MotionController::LoadMinuteAverageLog() {
  if (fs == nullptr) {
    return;
  }

  // Reset state
  inMemoryBufferCount = 0;
  diskEntryCount = 0;
  minuteAverageCount = 0;
  minuteAverageTotal = 0.0;
  minuteHeartRateTotal = 0;
  minuteHeartRateSampleCount = 0;

  lfs_file_t file;
  if (fs->FileOpen(&file, minuteAverageFile, LFS_O_RDONLY) != LFS_ERR_OK) {
    return;
  }

  DiskHeader header {};

  if (fs->FileRead(&file, reinterpret_cast<uint8_t*>(&header), sizeof(header)) != sizeof(header)) {
    fs->FileClose(&file);
    return;
  }

  // Only support current version - delete incompatible files
  if (header.version != minuteAverageLogVersion) {
    fs->FileClose(&file);
    fs->FileDelete(minuteAverageFile);
    return;
  }

  // Load cached totals from header
  diskEntryCount = std::min(static_cast<size_t>(header.count), maxDiskLogEntries);
  minuteAverageCount = diskEntryCount;
  minuteAverageTotal = header.totalCounts;
  minuteHeartRateTotal = header.totalHeartRate;
  minuteHeartRateSampleCount = header.heartRateSampleCount;

  fs->FileClose(&file);
}

size_t MotionController::GetStoredEntryCount() const {
  return minuteAverageCount;
}

bool MotionController::ReadStoredEntry(size_t index, MinuteEntryData& entry) {
  if (index >= minuteAverageCount) {
    return false;
  }

  // Check if entry is in disk storage
  if (index < diskEntryCount) {
    if (fs == nullptr || !storageAccessible) {
      return false;
    }

    ScopedFile file(fs);
    if (!file.Open(minuteAverageFile, LFS_O_RDONLY)) {
      return false;
    }

    // Seek to the entry position (skip header)
    if (file.Seek(sizeof(DiskHeader) + index * sizeof(DiskEntry)) < 0) {
      return false;
    }

    DiskEntry diskEntry {};
    if (file.Read(&diskEntry, sizeof(diskEntry)) != sizeof(diskEntry)) {
      return false;
    }

    entry.counts = diskEntry.counts;
    entry.heartRate = diskEntry.heartRate;
    entry.timestamp = diskEntry.timestamp;
    return true;
  }

  // Entry is in the in-memory buffer
  size_t bufferIndex = index - diskEntryCount;
  if (bufferIndex >= inMemoryBufferCount) {
    return false;
  }

  entry.counts = inMemoryBuffer[bufferIndex].counts;
  entry.heartRate = inMemoryBuffer[bufferIndex].heartRate;
  entry.timestamp = inMemoryBuffer[bufferIndex].timestamp;
  return true;
}

void MotionController::FlushAndClearStoredData() {
  // First flush any pending in-memory data to disk
  if (inMemoryBufferCount > 0 && storageAccessible) {
    FlushBufferToDisk();
  }

  // Then clear everything
  ClearMinuteAverageLog();
}

void MotionController::FlushToStorage() {
  // This method is called by SystemTask during periodic wake-ups
  // It temporarily marks storage as accessible to flush the buffer
  if (inMemoryBufferCount == 0) {
    return;
  }

  bool wasAccessible = storageAccessible;
  storageAccessible = true;
  FlushBufferToDisk();
  storageAccessible = wasAccessible;
}
