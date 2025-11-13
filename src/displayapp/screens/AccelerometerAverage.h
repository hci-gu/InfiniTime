#pragma once

#include <lvgl/lvgl.h>
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
        lv_obj_t* averageLabel = nullptr;
        lv_obj_t* deleteButton = nullptr;
        lv_obj_t* deleteButtonLabel = nullptr;
        lv_task_t* taskRefresh = nullptr;

        void DeleteLoggedMinutes();
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
