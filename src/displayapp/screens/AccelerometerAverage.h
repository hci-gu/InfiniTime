#pragma once

#include <array>
#include <lvgl/lvgl.h>
#include "components/datetime/DateTimeController.h"
#include "components/motion/MotionController.h"
#include "displayapp/Controllers.h"
#include "displayapp/apps/Apps.h"
#include "displayapp/screens/Screen.h"
#include "displayapp/screens/Symbols.h"

namespace Pinetime {
  namespace Applications {
    namespace Screens {

      class AccelerometerAverage : public Screen {
      public:
        explicit AccelerometerAverage(Controllers::MotionController& motionController,
                                     Controllers::DateTime& dateTimeController);
        ~AccelerometerAverage() override;

        void Refresh() override;

      private:
        Controllers::MotionController& motionController;
        Controllers::DateTime& dateTimeController;
        lv_obj_t* countLabel = nullptr;
        lv_obj_t* averageLabel = nullptr;
        lv_obj_t* historyTitleLabel = nullptr;
        static constexpr size_t historyEntryCount = 3;
        std::array<lv_obj_t*, historyEntryCount> historyLabels {};
        lv_obj_t* deleteButton = nullptr;
        lv_obj_t* deleteButtonLabel = nullptr;
        lv_task_t* taskRefresh = nullptr;

        void DeleteLoggedMinutes();
        void UpdateRecentHistory();
        static void DeleteButtonEventHandler(lv_obj_t* obj, lv_event_t event);
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
