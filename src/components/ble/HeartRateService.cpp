#include "components/ble/HeartRateService.h"
#include "components/heartrate/HeartRateController.h"
#include "components/motion/MotionController.h"
#include "components/ble/NimbleController.h"
#include <host/ble_hs_mbuf.h>
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
    return AppendAccelAverageLog(context->om);
  }
  return 0;
}

void HeartRateService::OnNewHeartRateValue(uint8_t heartRateValue) {
  static_cast<void>(heartRateValue);
  if (!heartRateMeasurementNotificationEnable)
    return;

  auto* om = ble_hs_mbuf_att_pkt();
  if (om == nullptr) {
    return;
  }

  if (AppendAccelAverageLog(om) != 0) {
    os_mbuf_free_chain(om);
    return;
  }

  uint16_t connectionHandle = nimble.connHandle();

  if (connectionHandle == 0 || connectionHandle == BLE_HS_CONN_HANDLE_NONE) {
    return;
  }

  ble_gattc_notify_custom(connectionHandle, heartRateMeasurementHandle, om);
}

void HeartRateService::SubscribeNotification(uint16_t attributeHandle) {
  if (attributeHandle == heartRateMeasurementHandle)
    heartRateMeasurementNotificationEnable = true;
}

void HeartRateService::UnsubscribeNotification(uint16_t attributeHandle) {
  if (attributeHandle == heartRateMeasurementHandle)
    heartRateMeasurementNotificationEnable = false;
}

int HeartRateService::AppendAccelAverageLog(struct os_mbuf* buffer) {
  const auto count = static_cast<uint32_t>(motionController.LoggedMinuteCount());
  if (os_mbuf_append(buffer, &count, sizeof(count)) != 0) {
    return BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  for (size_t i = 0; i < count; ++i) {
    const auto entry = motionController.GetMinuteAverageEntry(i);
    if (os_mbuf_append(buffer, &entry.timestamp, sizeof(entry.timestamp)) != 0) {
      return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (os_mbuf_append(buffer, &entry.acceleration, sizeof(entry.acceleration)) != 0) {
      return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (os_mbuf_append(buffer, &entry.heartRate, sizeof(entry.heartRate)) != 0) {
      return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
  }

  return 0;
}
