#include "components/motion/MotionController.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <task.h>

#include "utility/Math.h"

using namespace Pinetime::Controllers;

namespace {
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
  AddAccelerationSample(time, magnitude);

  int32_t deltaSteps = nbSteps - oldSteps;
  if (deltaSteps > 0) {
    currentTripSteps += deltaSteps;
  }
  SetSteps(Days::Today, nbSteps);
}

int32_t MotionController::AverageAccelerationLastMinute() {
  TickType_t now = xTaskGetTickCount();
  return AverageAccelerationLastMinuteInternal(now);
}

int32_t MotionController::LoggedMinutesAverage() const {
  if (minuteAverageCount == 0) {
    return 0;
  }

  return static_cast<int32_t>(minuteAverageTotal / static_cast<int64_t>(minuteAverageCount));
}

int32_t MotionController::LoggedMinutesHeartRateAverage() const {
  if (minuteHeartRateSampleCount == 0) {
    return 0;
  }

  return static_cast<int32_t>(minuteHeartRateTotal / static_cast<int64_t>(minuteHeartRateSampleCount));
}

bool MotionController::GetLoggedMinuteEntry(size_t relativeIndex, MinuteAverageEntry& entry) const {
  if (relativeIndex >= minuteAverageCount) {
    return false;
  }

  const size_t index = (minuteAverageStart + relativeIndex) % minuteAverageLogSize;
  entry.timestamp = minuteAverageTimestamps[index];
  entry.acceleration = minuteAccelerationAverages[index];
  entry.heartRate = minuteHeartRateAverages[index];
  return true;
}

void MotionController::ClearMinuteAverageLog() {
  minuteAverageStart = 0;
  minuteAverageCount = 0;
  minuteAverageTotal = 0;
  minuteHeartRateTotal = 0;
  minuteHeartRateSampleCount = 0;
  std::fill(minuteAverageTimestamps.begin(), minuteAverageTimestamps.end(), 0);
  std::fill(minuteAccelerationAverages.begin(), minuteAccelerationAverages.end(), 0);
  std::fill(minuteHeartRateAverages.begin(), minuteHeartRateAverages.end(), 0);
  minuteAverageDirty = true;
  MaybePersistMinuteAverageLog();
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

void MotionController::Init(Pinetime::Drivers::Bma421::DeviceTypes types, Pinetime::Controllers::FS& fsController) {
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
  storageAccessible = true;
  LoadMinuteAverageLog();
  lastLoggedMinuteTick = xTaskGetTickCount();
}

void MotionController::OnStorageWake() {
  storageAccessible = true;
  MaybePersistMinuteAverageLog();
}

void MotionController::OnStorageSleep() {
  storageAccessible = false;
}

void MotionController::AddAccelerationSample(TickType_t timestamp, int32_t magnitude) {
  if (accelSampleCount == accelSamplesWindow) {
    accelSampleTotal -= accelSamples[accelSampleTail].magnitude;
    accelSampleTail = (accelSampleTail + 1) % accelSamplesWindow;
    accelSampleCount--;
  }

  accelSamples[accelSampleHead].timestamp = timestamp;
  accelSamples[accelSampleHead].magnitude = magnitude;
  accelSampleHead = (accelSampleHead + 1) % accelSamplesWindow;

  accelSampleCount++;
  accelSampleTotal += magnitude;

  PruneOldAccelerationSamples(timestamp);
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

void MotionController::PruneOldAccelerationSamples(TickType_t currentTimestamp) {
  constexpr TickType_t window = configTICK_RATE_HZ * 60;

  while (accelSampleCount > 0) {
    const auto& oldest = accelSamples[accelSampleTail];
    TickType_t age = currentTimestamp - oldest.timestamp;
    if (age <= window) {
      break;
    }

    accelSampleTotal -= oldest.magnitude;
    accelSampleTail = (accelSampleTail + 1) % accelSamplesWindow;
    accelSampleCount--;
  }
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

int32_t MotionController::AverageAccelerationLastMinuteInternal(TickType_t currentTimestamp) {
  PruneOldAccelerationSamples(currentTimestamp);

  if (accelSampleCount == 0) {
    return 0;
  }

  return static_cast<int32_t>(accelSampleTotal / static_cast<int64_t>(accelSampleCount));
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

  int32_t accelerationAverage = AverageAccelerationLastMinuteInternal(timestamp);
  int32_t heartRateAverage = AverageHeartRateLastMinuteInternal(timestamp);

  if (accelerationAverage == 0 && accelSampleCount == 0) {
    lastLoggedMinuteTick = timestamp;
    return;
  }

  AppendMinuteAverage(timestamp, accelerationAverage, heartRateAverage);
  lastLoggedMinuteTick = timestamp;
}

void MotionController::AppendMinuteAverage(TickType_t timestamp, int32_t accelerationAverage, int32_t heartRateAverage) {
  const int32_t clampedHeartRate = std::clamp<int32_t>(
    heartRateAverage, 0, static_cast<int32_t>(std::numeric_limits<int16_t>::max()));
  const int16_t storedHeartRate = static_cast<int16_t>(clampedHeartRate);

  if (minuteAverageCount < minuteAverageLogSize) {
    size_t index = (minuteAverageStart + minuteAverageCount) % minuteAverageLogSize;
    minuteAverageTimestamps[index] = timestamp;
    minuteAccelerationAverages[index] = accelerationAverage;
    minuteHeartRateAverages[index] = storedHeartRate;
    minuteAverageCount++;
  } else {
    const auto oldestAcceleration = minuteAccelerationAverages[minuteAverageStart];
    const auto oldestHeartRate = minuteHeartRateAverages[minuteAverageStart];
    minuteAverageTotal -= oldestAcceleration;
    if (oldestHeartRate > 0 && minuteHeartRateSampleCount > 0) {
      minuteHeartRateTotal -= oldestHeartRate;
      minuteHeartRateSampleCount--;
    }
    minuteAverageTimestamps[minuteAverageStart] = timestamp;
    minuteAccelerationAverages[minuteAverageStart] = accelerationAverage;
    minuteHeartRateAverages[minuteAverageStart] = storedHeartRate;
    minuteAverageStart = (minuteAverageStart + 1) % minuteAverageLogSize;
  }

  minuteAverageTotal += accelerationAverage;
  if (storedHeartRate > 0) {
    minuteHeartRateTotal += storedHeartRate;
    minuteHeartRateSampleCount++;
  }
  minuteAverageDirty = true;
  MaybePersistMinuteAverageLog();
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

void MotionController::SaveMinuteAverageLog() {
  if (fs == nullptr) {
    return;
  }

  EnsureLogDirectory();

  lfs_file_t file;
  if (fs->FileOpen(&file, minuteAverageFile, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) != LFS_ERR_OK) {
    return;
  }

  struct Header {
    uint32_t count;
  } header {static_cast<uint32_t>(minuteAverageCount)};

  fs->FileWrite(&file, reinterpret_cast<const uint8_t*>(&header), sizeof(header));

  struct DiskEntry {
    TickType_t timestamp;
    int32_t acceleration;
    int32_t heartRate;
  };

  for (size_t i = 0; i < minuteAverageCount; ++i) {
    size_t index = (minuteAverageStart + i) % minuteAverageLogSize;
    DiskEntry entry {
      minuteAverageTimestamps[index], minuteAccelerationAverages[index], minuteHeartRateAverages[index]};
    fs->FileWrite(&file, reinterpret_cast<const uint8_t*>(&entry), sizeof(entry));
  }

  fs->FileClose(&file);
  minuteAverageDirty = false;
}

void MotionController::LoadMinuteAverageLog() {
  if (fs == nullptr) {
    return;
  }

  lfs_file_t file;
  if (fs->FileOpen(&file, minuteAverageFile, LFS_O_RDONLY) != LFS_ERR_OK) {
    return;
  }

  struct Header {
    uint32_t count;
  } header {};

  if (fs->FileRead(&file, reinterpret_cast<uint8_t*>(&header), sizeof(header)) != sizeof(header)) {
    fs->FileClose(&file);
    return;
  }

  minuteAverageStart = 0;
  minuteAverageCount = 0;
  minuteAverageTotal = 0;
  minuteHeartRateTotal = 0;
  minuteHeartRateSampleCount = 0;

  size_t count = std::min(static_cast<size_t>(header.count), minuteAverageLogSize);

  struct DiskEntry {
    TickType_t timestamp;
    int32_t acceleration;
    int32_t heartRate;
  } entry {};

  for (size_t i = 0; i < count; ++i) {
    if (fs->FileRead(&file, reinterpret_cast<uint8_t*>(&entry), sizeof(entry)) != sizeof(entry)) {
      break;
    }
    minuteAverageTimestamps[i] = entry.timestamp;
    minuteAccelerationAverages[i] = entry.acceleration;
    const int32_t clampedHeartRate = std::clamp<int32_t>(
      entry.heartRate, 0, static_cast<int32_t>(std::numeric_limits<int16_t>::max()));
    minuteHeartRateAverages[i] = static_cast<int16_t>(clampedHeartRate);
    minuteAverageTotal += entry.acceleration;
    if (clampedHeartRate > 0) {
      minuteHeartRateTotal += clampedHeartRate;
      minuteHeartRateSampleCount++;
    }
    minuteAverageCount++;
  }

  fs->FileClose(&file);
}

void MotionController::MaybePersistMinuteAverageLog() {
  if (!minuteAverageDirty || fs == nullptr || !storageAccessible) {
    return;
  }

  SaveMinuteAverageLog();
}
