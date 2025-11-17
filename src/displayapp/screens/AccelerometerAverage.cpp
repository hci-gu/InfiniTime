#include "displayapp/screens/AccelerometerAverage.h"

#include <FreeRTOS.h>
#include <array>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <task.h>

#include "displayapp/InfiniTimeTheme.h"

using namespace Pinetime::Applications::Screens;
using namespace Pinetime::Controllers;


AccelerometerAverage::AccelerometerAverage(Controllers::MotionController& motionController,
                                           Controllers::DateTime& dateTimeController)
  : motionController {motionController}, dateTimeController {dateTimeController} {
  countLabel = lv_label_create(lv_scr_act(), nullptr);
  lv_label_set_text_static(countLabel, "Minutes stored: 0");
  lv_obj_set_style_local_text_color(countLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, Colors::lightGray);
  lv_obj_align(countLabel, lv_scr_act(), LV_ALIGN_IN_TOP_MID, 0, 10);

  averageLabel = lv_label_create(lv_scr_act(), nullptr);
  lv_obj_set_style_local_text_font(averageLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &jetbrains_mono_42);
  lv_obj_set_style_local_text_color(averageLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
  lv_label_set_text_static(averageLabel, "Avg accel: 0 mg\nAvg HR: -- bpm");
  lv_label_set_align(averageLabel, LV_LABEL_ALIGN_CENTER);
  lv_obj_align(averageLabel, nullptr, LV_ALIGN_CENTER, 0, 0);

  historyTitleLabel = lv_label_create(lv_scr_act(), nullptr);
  lv_obj_set_style_local_text_color(historyTitleLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, Colors::lightGray);
  lv_label_set_text_static(historyTitleLabel, "Latest minutes (HH:MM  HR  Accel)");
  lv_obj_align(historyTitleLabel, averageLabel, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);

  for (size_t i = 0; i < historyLabels.size(); ++i) {
    historyLabels[i] = lv_label_create(lv_scr_act(), nullptr);
    lv_label_set_text(historyLabels[i], "--:--   -- bpm   -- mg");
    if (i == 0) {
      lv_obj_align(historyLabels[i], historyTitleLabel, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);
    } else {
      lv_obj_align(historyLabels[i], historyLabels[i - 1], LV_ALIGN_OUT_BOTTOM_MID, 0, 2);
    }
  }

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

  auto accelAverage = motionController.LoggedMinutesAverage();
  if (motionController.HasLoggedHeartRateAverage()) {
    auto heartRateAverage = motionController.LoggedMinutesHeartRateAverage();
    lv_label_set_text_fmt(averageLabel,
                          "Avg accel: %ld mg\nAvg HR: %ld bpm",
                          static_cast<long>(accelAverage),
                          static_cast<long>(heartRateAverage));
  } else {
    lv_label_set_text_fmt(averageLabel, "Avg accel: %ld mg\nAvg HR: -- bpm", static_cast<long>(accelAverage));
  }
  lv_obj_align(averageLabel, nullptr, LV_ALIGN_CENTER, 0, 0);
  UpdateRecentHistory();
}

void AccelerometerAverage::DeleteLoggedMinutes() {
  motionController.ClearMinuteAverageLog();
  Refresh();
}

void AccelerometerAverage::UpdateRecentHistory() {
  std::array<MotionController::LoggedMinuteEntry, historyEntryCount> recentMinutes {};
  const auto available = motionController.GetRecentLoggedMinutes(recentMinutes);
  const auto nowTicks = xTaskGetTickCount();
  const auto nowDateTime = dateTimeController.CurrentDateTime();

  for (size_t i = 0; i < historyLabels.size(); ++i) {
    if (historyLabels[i] == nullptr) {
      continue;
    }

    if (i >= available) {
      lv_label_set_text(historyLabels[i], "--:--   -- bpm   -- mg");
      if (i == 0) {
        lv_obj_align(historyLabels[i], historyTitleLabel, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);
      } else {
        lv_obj_align(historyLabels[i], historyLabels[i - 1], LV_ALIGN_OUT_BOTTOM_MID, 0, 2);
      }
      continue;
    }

    const auto& entry = recentMinutes[i];
    const TickType_t tickDelta = nowTicks - entry.timestamp;
    auto entryTime = nowDateTime - std::chrono::seconds(tickDelta / configTICK_RATE_HZ);
    auto entryTimeT = std::chrono::system_clock::to_time_t(entryTime);
    auto entryTm = *std::localtime(&entryTimeT);

    char heartRateBuffer[12];
    if (entry.heartRate > 0) {
      snprintf(heartRateBuffer, sizeof(heartRateBuffer), "%3d bpm", entry.heartRate);
    } else {
      snprintf(heartRateBuffer, sizeof(heartRateBuffer), " -- bpm");
    }

    lv_label_set_text_fmt(historyLabels[i],
                          "%02d:%02d   %s   %ld mg",
                          entryTm.tm_hour,
                          entryTm.tm_min,
                          heartRateBuffer,
                          static_cast<long>(entry.acceleration));

    if (i == 0) {
      lv_obj_align(historyLabels[i], historyTitleLabel, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);
    } else {
      lv_obj_align(historyLabels[i], historyLabels[i - 1], LV_ALIGN_OUT_BOTTOM_MID, 0, 2);
    }
  }
}

void AccelerometerAverage::DeleteButtonEventHandler(lv_obj_t* obj, lv_event_t event) {
  if (event != LV_EVENT_CLICKED) {
    return;
  }

  auto* screen = static_cast<AccelerometerAverage*>(obj->user_data);
  screen->DeleteLoggedMinutes();
}
