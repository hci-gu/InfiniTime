#pragma once

#include <lvgl/lvgl.h>
#include <vector>
#include "displayapp/screens/Screen.h"
#include "displayapp/apps/Apps.h"
#include "displayapp/Controllers.h"
#include "displayapp/screens/Symbols.h"
#include "components/motion/MotionController.h"
#include "components/datetime/DateTimeController.h"

namespace Pinetime {
  namespace Applications {
    namespace Screens {

      class AccelerometerAverage : public Screen {
      public:
        AccelerometerAverage(Controllers::MotionController& motionController, Controllers::DateTime& dateTimeController);
        ~AccelerometerAverage() override;

        void Refresh() override;

      private:
        Controllers::MotionController& motionController;
        Controllers::DateTime& dateTimeController;
        lv_obj_t* countLabel = nullptr;
        lv_obj_t* averageLabel = nullptr;
        lv_obj_t* minuteList = nullptr;
        lv_obj_t* listPlaceholder = nullptr;
        lv_obj_t* deleteButton = nullptr;
        lv_obj_t* deleteButtonLabel = nullptr;
        lv_task_t* taskRefresh = nullptr;
        std::vector<lv_obj_t*> minuteRowLabels;

        void DeleteLoggedMinutes();
        static void DeleteButtonEventHandler(lv_obj_t* obj, lv_event_t event);
        void BuildMinuteList(size_t storedMinutes);
        void UpdateMinuteRowTexts();
        void ClearMinuteRows();
        void UpdatePlaceholderVisibility(size_t storedMinutes);
      };
    }

    template <>
    struct AppTraits<Apps::AccelAvg> {
      static constexpr Apps app = Apps::AccelAvg;
      static constexpr const char* icon = Screens::Symbols::tachometer;

      static Screens::Screen* Create(AppControllers& controllers) {
        return new Screens::AccelerometerAverage(controllers.motionController, controllers.dateTimeController);
      }

      static bool IsAvailable(Pinetime::Controllers::FS& /*filesystem*/) {
        return true;
      }
    };
  }
}
