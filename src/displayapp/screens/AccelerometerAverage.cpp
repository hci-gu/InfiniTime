#include "displayapp/screens/AccelerometerAverage.h"

#include "displayapp/InfiniTimeTheme.h"

using namespace Pinetime::Applications::Screens;


AccelerometerAverage::AccelerometerAverage(Controllers::MotionController& motionController,
                                           Controllers::DateTime& dateTimeController)
  : motionController {motionController}, dateTimeController {dateTimeController} {
  countLabel = lv_label_create(lv_scr_act(), nullptr);
  lv_label_set_text_static(countLabel, "Minutes stored: 0");
  lv_obj_set_style_local_text_color(countLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, Colors::lightGray);
  lv_obj_align(countLabel, lv_scr_act(), LV_ALIGN_IN_TOP_MID, 0, 10);

  averageLabel = lv_label_create(lv_scr_act(), nullptr);
  lv_obj_set_style_local_text_font(averageLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &jetbrains_mono_20);
  lv_obj_set_style_local_text_color(averageLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
  lv_label_set_text_static(averageLabel, "Avg accel: 0 mg\nAvg HR: -- bpm");
  lv_label_set_align(averageLabel, LV_LABEL_ALIGN_CENTER);
  lv_obj_align(averageLabel, lv_scr_act(), LV_ALIGN_IN_TOP_MID, 0, 40);

  historyTable = lv_table_create(lv_scr_act(), nullptr);
  lv_obj_set_size(historyTable, 230, 120);
  lv_obj_align(historyTable, nullptr, LV_ALIGN_IN_TOP_MID, 0, 80);
  lv_table_set_col_cnt(historyTable, 3);
  lv_table_set_row_cnt(historyTable, 1);
  lv_table_set_col_width(historyTable, 0, 70);
  lv_table_set_col_width(historyTable, 1, 90);
  lv_table_set_col_width(historyTable, 2, 60);
  lv_table_set_cell_value(historyTable, 0, 0, "Time");
  lv_table_set_cell_value(historyTable, 0, 1, "Accel");
  lv_table_set_cell_value(historyTable, 0, 2, "HR");
  lv_table_set_cell_align(historyTable, 0, 0, LV_LABEL_ALIGN_CENTER);
  lv_table_set_cell_align(historyTable, 0, 1, LV_LABEL_ALIGN_CENTER);
  lv_table_set_cell_align(historyTable, 0, 2, LV_LABEL_ALIGN_CENTER);

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
  auto currentHour = dateTimeController.Hours();
  auto currentMinute = dateTimeController.Minutes();
  if (storedMinutes != lastRenderedHistoryCount || currentMinute != lastRenderedMinute) {
    UpdateHistoryRows(currentHour, currentMinute, storedMinutes);
    lastRenderedHistoryCount = storedMinutes;
    lastRenderedMinute = currentMinute;
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

void AccelerometerAverage::UpdateHistoryRows(uint8_t currentHour, uint8_t currentMinute, size_t storedMinutes) {
  if (historyTable == nullptr) {
    return;
  }

  lv_table_set_row_cnt(historyTable, storedMinutes + 1);
  lv_table_set_cell_value(historyTable, 0, 0, "Time");
  lv_table_set_cell_value(historyTable, 0, 1, "Accel");
  lv_table_set_cell_value(historyTable, 0, 2, "HR");

  constexpr int minutesPerDay = 24 * 60;
  const int currentTotalMinutes = static_cast<int>(currentHour) * 60 + static_cast<int>(currentMinute);

  for (size_t i = 0; i < storedMinutes; ++i) {
    Controllers::MotionController::LoggedMinute entry {};
    if (!motionController.GetLoggedMinute(i, entry)) {
      continue;
    }

    int minutesValue = currentTotalMinutes - static_cast<int>(i);
    minutesValue %= minutesPerDay;
    if (minutesValue < 0) {
      minutesValue += minutesPerDay;
    }
    uint8_t rowHour = static_cast<uint8_t>(minutesValue / 60);
    uint8_t rowMinute = static_cast<uint8_t>(minutesValue % 60);

    lv_table_set_cell_value_fmt(historyTable, i + 1, 0, "%02u:%02u", rowHour, rowMinute);
    lv_table_set_cell_value_fmt(historyTable, i + 1, 1, "%ld", static_cast<long>(entry.acceleration));
    if (entry.heartRate > 0) {
      lv_table_set_cell_value_fmt(historyTable, i + 1, 2, "%d", static_cast<int>(entry.heartRate));
    } else {
      lv_table_set_cell_value(historyTable, i + 1, 2, "--");
    }
  }
}
