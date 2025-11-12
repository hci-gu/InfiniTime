#include "displayapp/screens/AccelerometerAverage.h"

#include "displayapp/InfiniTimeTheme.h"

using namespace Pinetime::Applications::Screens;

AccelerometerAverage::AccelerometerAverage(Controllers::MotionController& motionController)
  : motionController {motionController} {
  titleLabel = lv_label_create(lv_scr_act(), nullptr);
  lv_label_set_text_static(titleLabel, "Accel avg (1m)");
  lv_obj_set_style_local_text_color(titleLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, Colors::lightGray);
  lv_obj_align(titleLabel, lv_scr_act(), LV_ALIGN_IN_TOP_MID, 0, 10);

  averageLabel = lv_label_create(lv_scr_act(), nullptr);
  lv_obj_set_style_local_text_font(averageLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &jetbrains_mono_42);
  lv_obj_set_style_local_text_color(averageLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
  lv_label_set_text_static(averageLabel, "0 mg");
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
  auto average = motionController.AverageAccelerationLastMinute();
  lv_label_set_text_fmt(averageLabel, "%ld mg", static_cast<long>(average));
  lv_obj_align(averageLabel, nullptr, LV_ALIGN_CENTER, 0, 0);
}
