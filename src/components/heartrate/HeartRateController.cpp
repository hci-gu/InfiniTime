#include "components/heartrate/HeartRateController.h"
#include <heartratetask/HeartRateTask.h>
#include <systemtask/SystemTask.h>
#include "components/motion/MotionController.h"
#include <task.h>

using namespace Pinetime::Controllers;

void HeartRateController::Update(HeartRateController::States newState, uint8_t heartRate) {
  this->state = newState;
  this->heartRate = heartRate;

  if (motionController != nullptr && newState == States::Running && heartRate > 0) {
    motionController->AddHeartRateSample(xTaskGetTickCount(), heartRate);
  }
}

void HeartRateController::Enable() {
  if (task != nullptr) {
    state = States::NotEnoughData;
    task->PushMessage(Pinetime::Applications::HeartRateTask::Messages::Enable);
  }
}

void HeartRateController::Disable() {
  if (task != nullptr) {
    state = States::Stopped;
    task->PushMessage(Pinetime::Applications::HeartRateTask::Messages::Disable);
  }
}

void HeartRateController::SetHeartRateTask(Pinetime::Applications::HeartRateTask* task) {
  this->task = task;
}

void HeartRateController::SetMotionController(Pinetime::Controllers::MotionController* motionController) {
  this->motionController = motionController;
}
