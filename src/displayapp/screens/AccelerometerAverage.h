#pragma once

#include <lvgl/lvgl.h>
#include <limits>
#include "displayapp/screens/Screen.h"
#include "displayapp/apps/Apps.h"
#include "displayapp/Controllers.h"
#include "displayapp/screens/Symbols.h"
#include "components/motion/MotionController.h"

namespace Pinetime {
  namespace Applications {
    namespace Screens {

      class AccelerometerAverage : public Screen {
      public:
        AccelerometerAverage(Controllers::MotionController& motionController);
        AccelerometerAverage(Controllers::MotionController& motionController, Controllers::DateTime& dateTimeController);
        ~AccelerometerAverage() override;

        void Refresh() override;

      private:
        Controllers::MotionController& motionController;
        Controllers::DateTime* dateTimeController = nullptr;
        lv_obj_t* countLabel = nullptr;
        lv_obj_t* averageLabel = nullptr;
        lv_obj_t* deleteButton = nullptr;
        lv_obj_t* deleteButtonLabel = nullptr;
        lv_obj_t* historyTable = nullptr;
        size_t displayedMinuteCount = std::numeric_limits<size_t>::max();
        lv_task_t* taskRefresh = nullptr;

        void DeleteLoggedMinutes();
        static void DeleteButtonEventHandler(lv_obj_t* obj, lv_event_t event);
        void UpdateHistoryTable(size_t storedMinutes);
        void SetHistoryTableHeader();

        AccelerometerAverage(Controllers::MotionController& motionController,
                             Controllers::DateTime* dateTimeController);
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
