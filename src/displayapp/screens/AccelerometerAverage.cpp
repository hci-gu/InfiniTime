#include "displayapp/screens/AccelerometerAverage.h"

#include "displayapp/InfiniTimeTheme.h"

using namespace Pinetime::Applications::Screens;


AccelerometerAverage::AccelerometerAverage(Controllers::MotionController& motionController,
                                           Controllers::DateTime& dateTimeController)
  : motionController {motionController}, dateTimeController {dateTimeController} {
  auto* screen = lv_scr_act();

  countLabel = lv_label_create(screen, nullptr);
  lv_label_set_text_static(countLabel, "Minutes stored: 0");
  lv_obj_set_style_local_text_color(countLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, Colors::lightGray);
  lv_obj_align(countLabel, screen, LV_ALIGN_IN_TOP_MID, 0, 8);

  averageLabel = lv_label_create(screen, nullptr);
  lv_obj_set_style_local_text_font(averageLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &jetbrains_mono_42);
  lv_obj_set_style_local_text_color(averageLabel, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
  lv_label_set_text_static(averageLabel, "Avg accel: 0 mg\nAvg HR: -- bpm");
  lv_label_set_align(averageLabel, LV_LABEL_ALIGN_CENTER);
  lv_obj_align(averageLabel, screen, LV_ALIGN_IN_TOP_MID, 0, 52);

  minuteList = lv_page_create(screen, nullptr);
  lv_obj_set_size(minuteList, 220, 130);
  lv_obj_align(minuteList, screen, LV_ALIGN_IN_BOTTOM_MID, 0, -70);
  lv_page_set_scrollbar_mode(minuteList, LV_SCROLLBAR_MODE_AUTO);

  listPlaceholder = lv_label_create(lv_page_get_scrlable(minuteList), nullptr);
  lv_label_set_text_static(listPlaceholder, "No minute history yet");
  lv_label_set_align(listPlaceholder, LV_LABEL_ALIGN_CENTER);
  lv_obj_set_width(listPlaceholder, 200);
  lv_obj_set_style_local_text_color(listPlaceholder, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, Colors::lightGray);
  lv_obj_align(listPlaceholder, nullptr, LV_ALIGN_IN_TOP_MID, 0, 10);

  deleteButton = lv_btn_create(screen, nullptr);
  deleteButton->user_data = this;
  lv_obj_set_event_cb(deleteButton, DeleteButtonEventHandler);
  lv_obj_set_size(deleteButton, 190, 50);
  lv_obj_align(deleteButton, screen, LV_ALIGN_IN_BOTTOM_MID, 0, -8);
  deleteButtonLabel = lv_label_create(deleteButton, nullptr);
  lv_label_set_text_static(deleteButtonLabel, "Delete stored minutes");

  UpdatePlaceholderVisibility(0);
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

  if (storedMinutes != minuteRowLabels.size()) {
    BuildMinuteList(storedMinutes);
  } else {
    UpdateMinuteRowTexts();
    UpdatePlaceholderVisibility(storedMinutes);
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

void AccelerometerAverage::BuildMinuteList(size_t storedMinutes) {
  ClearMinuteRows();

  if (storedMinutes == 0) {
    UpdatePlaceholderVisibility(storedMinutes);
    return;
  }

  auto* scrollable = lv_page_get_scrlable(minuteList);
  minuteRowLabels.reserve(storedMinutes);

  for (size_t i = 0; i < storedMinutes; ++i) {
    auto* row = lv_label_create(scrollable, nullptr);
    lv_label_set_long_mode(row, LV_LABEL_LONG_EXPAND);
    lv_obj_set_width(row, lv_obj_get_width(minuteList) - 20);
    lv_label_set_align(row, LV_LABEL_ALIGN_LEFT);
    lv_obj_set_style_local_text_font(row, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &jetbrains_mono_bold_20);
    lv_obj_set_style_local_text_color(row, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_pad_bottom(row, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, 4);
    minuteRowLabels.push_back(row);
  }

  UpdateMinuteRowTexts();
  UpdatePlaceholderVisibility(storedMinutes);
}

void AccelerometerAverage::UpdateMinuteRowTexts() {
  if (minuteRowLabels.empty()) {
    return;
  }

  constexpr int minutesPerDay = 24 * 60;
  const int currentMinutesOfDay = static_cast<int>(dateTimeController.Hours()) * 60 + dateTimeController.Minutes();

  size_t index = 0;
  for (auto* label : minuteRowLabels) {
    Controllers::MotionController::MinuteAverageEntry entry {};
    if (!motionController.GetLoggedMinute(index, entry)) {
      lv_label_set_text_static(label, "");
      ++index;
      continue;
    }

    uint16_t minutesOfDay = entry.minuteOfDay;
    if (minutesOfDay >= minutesPerDay) {
      int fallback = currentMinutesOfDay - static_cast<int>(index) - 1;
      while (fallback < 0) {
        fallback += minutesPerDay;
      }
      minutesOfDay = static_cast<uint16_t>(fallback % minutesPerDay);
    }

    const uint16_t hour = minutesOfDay / 60;
    const uint16_t minute = minutesOfDay % 60;

    if (entry.heartRate > 0) {
      lv_label_set_text_fmt(label,
                            "%02u:%02u  %ld mg  %d bpm",
                            static_cast<unsigned>(hour),
                            static_cast<unsigned>(minute),
                            static_cast<long>(entry.acceleration),
                            static_cast<int>(entry.heartRate));
    } else {
      lv_label_set_text_fmt(label,
                            "%02u:%02u  %ld mg  -- bpm",
                            static_cast<unsigned>(hour),
                            static_cast<unsigned>(minute),
                            static_cast<long>(entry.acceleration));
    }

    ++index;
  }
}

void AccelerometerAverage::ClearMinuteRows() {
  for (auto* label : minuteRowLabels) {
    lv_obj_del(label);
  }
  minuteRowLabels.clear();
}

void AccelerometerAverage::UpdatePlaceholderVisibility(size_t storedMinutes) {
  if (listPlaceholder == nullptr) {
    return;
  }

  const bool hidePlaceholder = storedMinutes != 0;
  lv_obj_set_hidden(listPlaceholder, hidePlaceholder);
}
