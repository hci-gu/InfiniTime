#include "displayapp/screens/AccelerometerAverage.h"

#include <cstdio>

#include "displayapp/InfiniTimeTheme.h"

using namespace Pinetime::Applications::Screens;


AccelerometerAverage::AccelerometerAverage(Controllers::MotionController& motionController)
  : motionController {motionController} {
  countLabel = lv_label_create(lv_scr_act(), nullptr);
  lv_label_set_text_static(countLabel, "Minutes stored: 0");
  lv_obj_set_style_local_text_color(countLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, Colors::lightGray);
  lv_obj_align(countLabel, lv_scr_act(), LV_ALIGN_IN_TOP_MID, 0, 10);

  historyPage = lv_page_create(lv_scr_act(), nullptr);
  lv_obj_set_size(historyPage, 220, 150);
  lv_obj_align(historyPage, lv_scr_act(), LV_ALIGN_IN_TOP_MID, 0, 40);
  lv_page_set_scrollbar_mode(historyPage, LV_SCROLLBAR_MODE_AUTO);

  historyLabel = lv_label_create(historyPage, nullptr);
  lv_label_set_long_mode(historyLabel, LV_LABEL_LONG_BREAK);
  lv_obj_set_width(historyLabel, lv_obj_get_width(historyPage) - 10);
  lv_label_set_align(historyLabel, LV_LABEL_ALIGN_LEFT);
  lv_obj_set_style_local_text_font(historyLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &jetbrains_mono_bold_20);
  lv_label_set_text_static(historyLabel, "");

  deleteButton = lv_btn_create(lv_scr_act(), nullptr);
  deleteButton->user_data = this;
  lv_obj_set_event_cb(deleteButton, DeleteButtonEventHandler);
  lv_obj_set_size(deleteButton, 190, 50);
  lv_obj_align(deleteButton, lv_scr_act(), LV_ALIGN_IN_BOTTOM_MID, 0, -10);
  deleteButtonLabel = lv_label_create(deleteButton, nullptr);
  lv_label_set_text_static(deleteButtonLabel, "Delete stored minutes");

  Refresh();
  taskRefresh = lv_task_create(RefreshTaskCallback, 1000, LV_TASK_PRIO_MID, this);
}

AccelerometerAverage::~AccelerometerAverage() {
  if (taskRefresh != nullptr) {
    lv_task_del(taskRefresh);
  }
  lv_obj_clean(lv_scr_act());
}

void AccelerometerAverage::Refresh() {
  auto storedMinutes = motionController.LoggedMinuteCount();
  lv_label_set_text_fmt(countLabel, "Minutes stored: %lu", static_cast<unsigned long>(storedMinutes));

  Controllers::MotionController::LoggedMinute newestEntry {};
  bool hasNewestEntry = storedMinutes > 0 && motionController.GetLoggedMinute(0, newestEntry);
  bool needsUpdate = storedMinutes != lastRenderedCount;

  if (hasNewestEntry != hasLastRenderedNewest) {
    needsUpdate = true;
  } else if (hasNewestEntry && hasLastRenderedNewest) {
    if (newestEntry.acceleration != lastRenderedNewest.acceleration ||
        newestEntry.heartRate != lastRenderedNewest.heartRate ||
        newestEntry.timeMinutes != lastRenderedNewest.timeMinutes) {
      needsUpdate = true;
    }
  }

  if (!needsUpdate) {
    return;
  }

  UpdateHistoryText(storedMinutes);
  lastRenderedCount = storedMinutes;
  hasLastRenderedNewest = hasNewestEntry;
  if (hasNewestEntry) {
    lastRenderedNewest = newestEntry;
  } else {
    lastRenderedNewest = {};
  }
}

void AccelerometerAverage::DeleteLoggedMinutes() {
  motionController.ClearMinuteAverageLog();
  Refresh();
}

void AccelerometerAverage::DeleteButtonEventHandler(lv_obj_t* obj, lv_event_t event) {
  if (event != LV_EVENT_CLICKED) {
    return;
  }

  auto* screen = static_cast<AccelerometerAverage*>(obj->user_data);
  screen->DeleteLoggedMinutes();
}

namespace {
  constexpr const char headerText[] = "Time   Accel   HR\n";
  constexpr const char emptyHistoryText[] = "No logged minutes yet\n";

  size_t LoggedMinuteLineLength(const Controllers::MotionController::LoggedMinute& entry) {
    const auto acceleration = static_cast<long>(entry.acceleration);
    if (entry.timeMinutes != Controllers::MotionController::UnknownLoggedMinuteTime) {
      const auto hours = entry.timeMinutes / 60;
      const auto minutes = entry.timeMinutes % 60;
      if (entry.heartRate > 0) {
        return static_cast<size_t>(std::snprintf(nullptr, 0, "%02u:%02u  %ld  %d\n", hours, minutes, acceleration, entry.heartRate));
      }
      return static_cast<size_t>(std::snprintf(nullptr, 0, "%02u:%02u  %ld  --\n", hours, minutes, acceleration));
    }

    if (entry.heartRate > 0) {
      return static_cast<size_t>(std::snprintf(nullptr, 0, "--:--  %ld  %d\n", acceleration, entry.heartRate));
    }
    return static_cast<size_t>(std::snprintf(nullptr, 0, "--:--  %ld  --\n", acceleration));
  }

  size_t WriteText(char* buffer, size_t capacity, size_t offset, const char* text) {
    if (offset >= capacity) {
      return capacity;
    }
    const size_t remaining = capacity - offset;
    int written = std::snprintf(buffer + offset, remaining, "%s", text);
    if (written < 0) {
      return offset;
    }
    const size_t consumed = static_cast<size_t>(written);
    if (consumed >= remaining) {
      return capacity - 1;
    }
    return offset + consumed;
  }

  size_t WriteLoggedMinuteLine(char* buffer,
                               size_t capacity,
                               size_t offset,
                               const Controllers::MotionController::LoggedMinute& entry) {
    if (offset >= capacity) {
      return capacity;
    }
    const size_t remaining = capacity - offset;
    const auto acceleration = static_cast<long>(entry.acceleration);
    int written = 0;
    if (entry.timeMinutes != Controllers::MotionController::UnknownLoggedMinuteTime) {
      const auto hours = entry.timeMinutes / 60;
      const auto minutes = entry.timeMinutes % 60;
      if (entry.heartRate > 0) {
        written = std::snprintf(buffer + offset, remaining, "%02u:%02u  %ld  %d\n", hours, minutes, acceleration, entry.heartRate);
      } else {
        written = std::snprintf(buffer + offset, remaining, "%02u:%02u  %ld  --\n", hours, minutes, acceleration);
      }
    } else {
      if (entry.heartRate > 0) {
        written = std::snprintf(buffer + offset, remaining, "--:--  %ld  %d\n", acceleration, entry.heartRate);
      } else {
        written = std::snprintf(buffer + offset, remaining, "--:--  %ld  --\n", acceleration);
      }
    }

    if (written < 0) {
      return offset;
    }

    const size_t consumed = static_cast<size_t>(written);
    if (consumed >= remaining) {
      return capacity - 1;
    }
    return offset + consumed;
  }
}

void AccelerometerAverage::UpdateHistoryText(size_t storedMinutes) {
  size_t requiredChars = sizeof(headerText);
  size_t linesToWrite = 0;

  if (storedMinutes == 0) {
    requiredChars += sizeof(emptyHistoryText);
  } else {
    for (size_t i = 0; i < storedMinutes; ++i) {
      Controllers::MotionController::LoggedMinute entry {};
      if (!motionController.GetLoggedMinute(i, entry)) {
        break;
      }
      requiredChars += LoggedMinuteLineLength(entry);
      linesToWrite++;
    }

    if (linesToWrite == 0) {
      requiredChars += sizeof(emptyHistoryText);
    }
  }

  auto newBuffer = std::make_unique<char[]>(requiredChars);
  size_t offset = 0;
  offset = WriteText(newBuffer.get(), requiredChars, offset, headerText);

  if (linesToWrite == 0) {
    WriteText(newBuffer.get(), requiredChars, offset, emptyHistoryText);
  } else {
    for (size_t i = 0; i < linesToWrite; ++i) {
      Controllers::MotionController::LoggedMinute entry {};
      if (!motionController.GetLoggedMinute(i, entry)) {
        break;
      }
      offset = WriteLoggedMinuteLine(newBuffer.get(), requiredChars, offset, entry);
    }
  }

  newBuffer[requiredChars - 1] = '\0';
  historyBuffer = std::move(newBuffer);
  lv_label_set_text_static(historyLabel, historyBuffer.get());
}
