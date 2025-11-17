#include "displayapp/screens/AccelerometerAverage.h"

#include "displayapp/InfiniTimeTheme.h"

using namespace Pinetime::Applications::Screens;


AccelerometerAverage::AccelerometerAverage(Controllers::MotionController& motionController)
  : motionController {motionController} {
  countLabel = lv_label_create(lv_scr_act(), nullptr);
  lv_label_set_text_static(countLabel, "Minutes stored: 0");
  lv_obj_set_style_local_text_color(countLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, Colors::lightGray);
  lv_obj_align(countLabel, lv_scr_act(), LV_ALIGN_IN_TOP_MID, 0, 10);

  historyTable = lv_table_create(lv_scr_act(), nullptr);
  lv_table_set_col_cnt(historyTable, 3);
  lv_table_set_col_width(historyTable, 0, 70);
  lv_table_set_col_width(historyTable, 1, 80);
  lv_table_set_col_width(historyTable, 2, 70);
  lv_table_set_row_cnt(historyTable, 1);
  lv_obj_set_size(historyTable, 220, 150);
  lv_obj_align(historyTable, lv_scr_act(), LV_ALIGN_IN_TOP_MID, 0, 40);

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

  lv_table_set_row_cnt(historyTable, static_cast<uint16_t>(storedMinutes + 1));
  lv_table_set_cell_value(historyTable, 0, 0, "Time");
  lv_table_set_cell_value(historyTable, 0, 1, "Accel");
  lv_table_set_cell_value(historyTable, 0, 2, "HR");

  for (size_t i = 0; i < storedMinutes; ++i) {
    Controllers::MotionController::LoggedMinute entry {};
    if (!motionController.GetLoggedMinute(i, entry)) {
      lv_table_set_row_cnt(historyTable, static_cast<uint16_t>(i + 1));
      break;
    }

    const uint16_t rowIndex = static_cast<uint16_t>(i + 1);
    if (entry.timeMinutes != Controllers::MotionController::UnknownLoggedMinuteTime) {
      const auto hours = entry.timeMinutes / 60;
      const auto minutes = entry.timeMinutes % 60;
      lv_table_set_cell_value_fmt(historyTable, rowIndex, 0, "%02u:%02u", hours, minutes);
    } else {
      lv_table_set_cell_value(historyTable, rowIndex, 0, "--:--");
    }
    lv_table_set_cell_value_fmt(historyTable, rowIndex, 1, "%ld", static_cast<long>(entry.acceleration));

    if (entry.heartRate > 0) {
      lv_table_set_cell_value_fmt(historyTable, rowIndex, 2, "%d", entry.heartRate);
    } else {
      lv_table_set_cell_value(historyTable, rowIndex, 2, "--");
    }
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
