#include "components/ble/HeartRateService.h"
#include "components/heartrate/HeartRateController.h"
#include "components/motion/MotionController.h"
#include "components/ble/NimbleController.h"
#include <nrf_log.h>

using namespace Pinetime::Controllers;

constexpr ble_uuid16_t HeartRateService::heartRateServiceUuid;
constexpr ble_uuid16_t HeartRateService::heartRateMeasurementUuid;

namespace {
  int HeartRateServiceCallback(uint16_t /*conn_handle*/, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt, void* arg) {
    auto* heartRateService = static_cast<HeartRateService*>(arg);
    return heartRateService->OnHeartRateRequested(attr_handle, ctxt);
  }
}

// TODO Refactoring - remove dependency to SystemTask
HeartRateService::HeartRateService(NimbleController& nimble,
                                   Controllers::HeartRateController& heartRateController,
                                   Controllers::MotionController& motionController)
  : nimble {nimble},
    heartRateController {heartRateController},
    motionController {motionController},
    characteristicDefinition {{.uuid = &heartRateMeasurementUuid.u,
                               .access_cb = HeartRateServiceCallback,
                               .arg = this,
                               .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                               .val_handle = &heartRateMeasurementHandle},
                              {0}},
    serviceDefinition {
      {/* Device Information Service */
       .type = BLE_GATT_SVC_TYPE_PRIMARY,
       .uuid = &heartRateServiceUuid.u,
       .characteristics = characteristicDefinition},
      {0},
    } {
  // TODO refactor to prevent this loop dependency (service depends on controller and controller depends on service)
  heartRateController.SetService(this);
}

void HeartRateService::Init() {
  int res = 0;
  res = ble_gatts_count_cfg(serviceDefinition);
  ASSERT(res == 0);

  res = ble_gatts_add_svcs(serviceDefinition);
  ASSERT(res == 0);
}

int HeartRateService::OnHeartRateRequested(uint16_t attributeHandle, ble_gatt_access_ctxt* context) {
  if (attributeHandle == heartRateMeasurementHandle) {
    NRF_LOG_INFO("HEARTRATE : handle = %d", heartRateMeasurementHandle);
    size_t minuteCount = 0;
    auto payload = BuildMinuteAveragePayload(minuteCount);

    if (payload.empty()) {
      return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    int res = os_mbuf_append(context->om, payload.data(), payload.size());
    return (res == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  return 0;
}

void HeartRateService::OnNewHeartRateValue(uint8_t /*heartRateValue*/) {
  if (!heartRateMeasurementNotificationEnable)
    return;

  size_t minuteCount = 0;
  auto payload = BuildMinuteAveragePayload(minuteCount);

  if (payload.empty()) {
    return;
  }

  const auto lastNotifiedCount = lastMinuteAverageCountNotified.load();
  if (minuteCount == lastNotifiedCount) {
    return;
  }

  auto* om = ble_hs_mbuf_from_flat(payload.data(), payload.size());

  uint16_t connectionHandle = nimble.connHandle();

  if (connectionHandle == 0 || connectionHandle == BLE_HS_CONN_HANDLE_NONE) {
    return;
  }

  lastMinuteAverageCountNotified = minuteCount;
  ble_gattc_notify_custom(connectionHandle, heartRateMeasurementHandle, om);
}

void HeartRateService::SubscribeNotification(uint16_t attributeHandle) {
  if (attributeHandle == heartRateMeasurementHandle) {
    heartRateMeasurementNotificationEnable = true;
    lastMinuteAverageCountNotified = 0;
  }
}

void HeartRateService::UnsubscribeNotification(uint16_t attributeHandle) {
  if (attributeHandle == heartRateMeasurementHandle)
    heartRateMeasurementNotificationEnable = false;
}

std::vector<uint8_t> HeartRateService::BuildMinuteAveragePayload(size_t& minuteCount) const {
  minuteCount = motionController.LoggedMinuteCount();

  struct __attribute__((packed)) MinuteAveragePacketEntry {
    uint32_t timestamp;
    int32_t acceleration;
    int16_t heartRate;
  };
  static_assert(sizeof(MinuteAveragePacketEntry) == 10, "MinuteAveragePacketEntry must remain packed");

  const size_t entryCount = minuteCount;
  const size_t payloadSize = sizeof(uint32_t) + entryCount * sizeof(MinuteAveragePacketEntry);
  std::vector<uint8_t> payload;
  payload.reserve(payloadSize);

  const uint32_t storedCount = static_cast<uint32_t>(entryCount);
  const auto* countPtr = reinterpret_cast<const uint8_t*>(&storedCount);
  payload.insert(payload.end(), countPtr, countPtr + sizeof(storedCount));

  Pinetime::Controllers::MotionController::MinuteAverageEntry entry {};
  for (size_t i = 0; i < entryCount; ++i) {
    if (!motionController.GetLoggedMinuteEntry(i, entry)) {
      break;
    }

    MinuteAveragePacketEntry packetEntry {static_cast<uint32_t>(entry.timestamp), entry.acceleration, entry.heartRate};
    const auto* entryPtr = reinterpret_cast<const uint8_t*>(&packetEntry);
    payload.insert(payload.end(), entryPtr, entryPtr + sizeof(packetEntry));
  }

  return payload;
}
