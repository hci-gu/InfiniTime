#include "displayapp/screens/AccelerometerAverage.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>

#include "components/datetime/DateTimeController.h"
#include "displayapp/InfiniTimeTheme.h"

extern lv_font_t jetbrains_mono_bold_20;

using namespace Pinetime::Applications::Screens;


AccelerometerAverage::AccelerometerAverage(Controllers::MotionController& motionController)
  : AccelerometerAverage {motionController, nullptr} {
}

AccelerometerAverage::AccelerometerAverage(Controllers::MotionController& motionController,
                                           Controllers::DateTime& dateTimeController)
  : AccelerometerAverage {motionController, &dateTimeController} {
}

AccelerometerAverage::AccelerometerAverage(Controllers::MotionController& motionController,
                                           Controllers::DateTime* dateTimeController)
  : motionController {motionController}, dateTimeController {dateTimeController} {
  countLabel = lv_label_create(lv_scr_act(), nullptr);
  lv_label_set_text_static(countLabel, "Minutes stored: 0");
  lv_obj_set_style_local_text_color(countLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, Colors::lightGray);
  lv_obj_align(countLabel, lv_scr_act(), LV_ALIGN_IN_TOP_MID, 0, 6);

  averageLabel = lv_label_create(lv_scr_act(), nullptr);
  lv_obj_set_style_local_text_font(averageLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &jetbrains_mono_bold_20);
  lv_obj_set_style_local_text_color(averageLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
  lv_label_set_text_static(averageLabel, "Avg accel: 0 mg\nAvg HR: -- bpm");
  lv_label_set_align(averageLabel, LV_LABEL_ALIGN_CENTER);
  lv_obj_align(averageLabel, countLabel, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);

  deleteButton = lv_btn_create(lv_scr_act(), nullptr);
  deleteButton->user_data = this;
  lv_obj_set_event_cb(deleteButton, DeleteButtonEventHandler);
  lv_obj_set_size(deleteButton, 190, 50);
  lv_obj_align(deleteButton, lv_scr_act(), LV_ALIGN_IN_BOTTOM_MID, 0, -10);
  deleteButtonLabel = lv_label_create(deleteButton, nullptr);
  lv_label_set_text_static(deleteButtonLabel, "Delete stored minutes");

  historyTable = lv_table_create(lv_scr_act(), nullptr);
  lv_obj_set_size(historyTable, 220, 100);
  lv_table_set_col_cnt(historyTable, 3);
  lv_table_set_row_cnt(historyTable, 1);
  lv_table_set_col_width(historyTable, 0, 60);
  lv_table_set_col_width(historyTable, 1, 100);
  lv_table_set_col_width(historyTable, 2, 60);
  lv_obj_set_style_local_pad_all(historyTable, LV_TABLE_PART_CELL1, LV_STATE_DEFAULT, 4);
  lv_obj_set_style_local_border_color(historyTable, LV_TABLE_PART_CELL1, LV_STATE_DEFAULT, Colors::lightGray);
  lv_obj_align(historyTable, deleteButton, LV_ALIGN_OUT_TOP_MID, 0, -10);
  SetHistoryTableHeader();

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
  if (storedMinutes > maxHistoryRows) {
    lv_label_set_text_fmt(countLabel,
                          "Minutes stored: %lu (showing last %u)",
                          static_cast<unsigned long>(storedMinutes),
                          static_cast<unsigned>(maxHistoryRows));
  } else {
    lv_label_set_text_fmt(countLabel, "Minutes stored: %lu", static_cast<unsigned long>(storedMinutes));
  }

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
  lv_obj_align(averageLabel, countLabel, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);

  UpdateHistoryTable(storedMinutes);
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

void AccelerometerAverage::UpdateHistoryTable(size_t storedMinutes) {
  if (historyTable == nullptr || storedMinutes == displayedMinuteCount) {
    return;
  }

  displayedMinuteCount = storedMinutes;
  const size_t rowsToDisplay = std::min(storedMinutes, maxHistoryRows);
  lv_table_set_row_cnt(historyTable, rowsToDisplay + 1);
  SetHistoryTableHeader();

  if (storedMinutes == 0 || rowsToDisplay == 0) {
    return;
  }

  const auto hasDateTime = dateTimeController != nullptr;
  std::chrono::time_point<std::chrono::system_clock, std::chrono::system_clock::duration> now {};
  if (hasDateTime) {
    now = dateTimeController->CurrentDateTime();
  }

  const size_t oldestIndex = storedMinutes - rowsToDisplay;
  for (size_t row = 0; row < rowsToDisplay; ++row) {
    size_t logIndex = oldestIndex + row;
    size_t tableRow = row + 1;
    auto entry = motionController.LoggedMinuteAt(logIndex);
    char timeBuffer[6] = {'-', '-', ':', '-', '-', '\0'};
    if (hasDateTime) {
      size_t minutesAgo = storedMinutes - logIndex;
      auto entryTime = now - std::chrono::minutes(static_cast<int64_t>(minutesAgo));
      auto systemEntryTime =
        std::chrono::time_point_cast<std::chrono::system_clock::duration>(entryTime);
      std::time_t entryTimeT = std::chrono::system_clock::to_time_t(systemEntryTime);

      if (auto* tmInfo = std::localtime(&entryTimeT); tmInfo != nullptr) {
        snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d", tmInfo->tm_hour, tmInfo->tm_min);
      }
    }
    lv_table_set_cell_value(historyTable, tableRow, 0, timeBuffer);

    char accelBuffer[16];
    snprintf(accelBuffer, sizeof(accelBuffer), "%ld", static_cast<long>(entry.acceleration));
    lv_table_set_cell_value(historyTable, tableRow, 1, accelBuffer);

    if (entry.heartRate > 0) {
      char heartBuffer[8];
      snprintf(heartBuffer, sizeof(heartBuffer), "%d", entry.heartRate);
      lv_table_set_cell_value(historyTable, tableRow, 2, heartBuffer);
    } else {
      lv_table_set_cell_value(historyTable, tableRow, 2, "--");
    }
  }
}

void AccelerometerAverage::SetHistoryTableHeader() {
  if (historyTable == nullptr) {
    return;
  }

  lv_table_set_cell_value(historyTable, 0, 0, "Time");
  lv_table_set_cell_value(historyTable, 0, 1, "Accel");
  lv_table_set_cell_value(historyTable, 0, 2, "HR");
}
