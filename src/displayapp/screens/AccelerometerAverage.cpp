#include "displayapp/screens/AccelerometerAverage.h"

#include "displayapp/InfiniTimeTheme.h"

using namespace Pinetime::Applications::Screens;

AccelerometerAverage::AccelerometerAverage(Controllers::MotionController& motionController)
  : motionController {motionController} {
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
}
