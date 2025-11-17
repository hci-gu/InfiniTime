#pragma once

#include <lvgl/lvgl.h>
#include <limits>
#include <memory>

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
        explicit AccelerometerAverage(Controllers::MotionController& motionController);
        ~AccelerometerAverage() override;

        void Refresh() override;

      private:
        Controllers::MotionController& motionController;
        lv_obj_t* countLabel = nullptr;
        lv_obj_t* historyPage = nullptr;
        lv_obj_t* historyLabel = nullptr;
        lv_obj_t* deleteButton = nullptr;
        lv_obj_t* deleteButtonLabel = nullptr;
        lv_task_t* taskRefresh = nullptr;

        std::unique_ptr<char[]> historyBuffer;
        size_t lastRenderedCount = std::numeric_limits<size_t>::max();
        bool hasLastRenderedNewest = false;
        Controllers::MotionController::LoggedMinute lastRenderedNewest = {};

        void DeleteLoggedMinutes();
        void UpdateHistoryText(size_t storedMinutes);
        static void DeleteButtonEventHandler(lv_obj_t* obj, lv_event_t event);
      };
    }

    template <>
    struct AppTraits<Apps::AccelAvg> {
      static constexpr Apps app = Apps::AccelAvg;
      static constexpr const char* icon = Screens::Symbols::tachometer;

      static Screens::Screen* Create(AppControllers& controllers) {
        return new Screens::AccelerometerAverage(controllers.motionController);
      }

      static bool IsAvailable(Pinetime::Controllers::FS& /*filesystem*/) {
        return true;
      }
    };
  }
}
