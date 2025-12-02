#include "components/ble/HeartRateService.h"
#include "components/motion/MotionController.h"
#include "components/ble/NimbleController.h"
#include <nrf_log.h>
#include <algorithm>
#include <task.h>

using namespace Pinetime::Controllers;

constexpr ble_uuid128_t HeartRateService::accelDataServiceUuid;
constexpr ble_uuid128_t HeartRateService::transferCharUuid;

namespace {
  int AccelDataServiceCallback(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt, void* arg) {
    auto* service = static_cast<HeartRateService*>(arg);
    return service->OnDataRequested(conn_handle, attr_handle, ctxt);
  }
}

HeartRateService::HeartRateService(NimbleController& nimble, Controllers::MotionController& motionController)
  : nimble {nimble},
    motionController {motionController},
    characteristicDefinition {{.uuid = &transferCharUuid.u,
                               .access_cb = AccelDataServiceCallback,
                               .arg = this,
                               .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                               .val_handle = &transferCharHandle},
                              {0}},
    serviceDefinition {
      {.type = BLE_GATT_SVC_TYPE_PRIMARY,
       .uuid = &accelDataServiceUuid.u,
       .characteristics = characteristicDefinition},
      {0},
    } {
}

void HeartRateService::Init() {
  int res = 0;
  res = ble_gatts_count_cfg(serviceDefinition);
  ASSERT(res == 0);

  res = ble_gatts_add_svcs(serviceDefinition);
  ASSERT(res == 0);
}

int HeartRateService::OnDataRequested(uint16_t connectionHandle, uint16_t attributeHandle, ble_gatt_access_ctxt* context) {
  if (attributeHandle == transferCharHandle) {
    if (context->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
      return HandleCommand(connectionHandle, context->om);
    } else if (context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
      // Return entry count on read
      uint32_t count = static_cast<uint32_t>(motionController.GetStoredEntryCount());
      int res = os_mbuf_append(context->om, &count, sizeof(count));
      return (res == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
  }
  return 0;
}

int HeartRateService::HandleCommand(uint16_t connectionHandle, os_mbuf* om) {
  if (om->om_len < 1) {
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }

  auto command = static_cast<Commands>(om->om_data[0]);
  NRF_LOG_INFO("[ACCEL_DATA] Command: %d", static_cast<int>(command));

  switch (command) {
    case Commands::GET_COUNT:
      SendCountResponse(connectionHandle);
      break;

    case Commands::READ_ENTRIES: {
      if (om->om_len < sizeof(ReadEntriesRequest)) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
      }
      auto* request = reinterpret_cast<ReadEntriesRequest*>(om->om_data);
      uint8_t count = std::min(request->count, static_cast<uint8_t>(20)); // Max 20 entries per packet
      SendEntries(connectionHandle, request->startIndex, count);
      break;
    }

    case Commands::CLEAR_DATA:
      motionController.FlushAndClearStoredData();
      SendClearResponse(connectionHandle, true);
      break;

    default:
      NRF_LOG_WARNING("[ACCEL_DATA] Unknown command: %d", static_cast<int>(command));
      return BLE_ATT_ERR_UNLIKELY;
  }

  return 0;
}

void HeartRateService::SendCountResponse(uint16_t connectionHandle) {
  CountResponse response {
    .command = static_cast<uint8_t>(Commands::GET_COUNT_RESPONSE),
    .status = 0x01,
    .count = static_cast<uint32_t>(motionController.GetStoredEntryCount())
  };

  auto* om = ble_hs_mbuf_from_flat(&response, sizeof(response));
  if (om != nullptr) {
    ble_gattc_notify_custom(connectionHandle, transferCharHandle, om);
  }
}

void HeartRateService::SendEntries(uint16_t connectionHandle, uint32_t startIndex, uint8_t count) {
  uint32_t totalCount = static_cast<uint32_t>(motionController.GetStoredEntryCount());

  if (startIndex >= totalCount) {
    // Send empty response
    ReadEntriesHeader header {
      .command = static_cast<uint8_t>(Commands::READ_ENTRIES_RESPONSE),
      .status = 0x01,
      .startIndex = startIndex,
      .totalCount = totalCount,
      .entriesInPacket = 0
    };
    auto* om = ble_hs_mbuf_from_flat(&header, sizeof(header));
    if (om != nullptr) {
      ble_gattc_notify_custom(connectionHandle, transferCharHandle, om);
    }
    return;
  }

  // Calculate how many entries we can actually send
  uint32_t availableEntries = totalCount - startIndex;
  uint8_t entriesToSend = static_cast<uint8_t>(std::min(static_cast<uint32_t>(count), availableEntries));

  // Build response header
  ReadEntriesHeader header {
    .command = static_cast<uint8_t>(Commands::READ_ENTRIES_RESPONSE),
    .status = 0x01,
    .startIndex = startIndex,
    .totalCount = totalCount,
    .entriesInPacket = entriesToSend
  };

  auto* om = ble_hs_mbuf_from_flat(&header, sizeof(header));
  if (om == nullptr) {
    return;
  }

  // Append entry data
  for (uint8_t i = 0; i < entriesToSend; ++i) {
    MotionController::MinuteEntryData entryData {};
    if (motionController.ReadStoredEntry(startIndex + i, entryData)) {
      EntryData entry {
        .counts = entryData.counts,
        .heartRate = entryData.heartRate,
        .timestamp = entryData.timestamp
      };
      os_mbuf_append(om, &entry, sizeof(entry));
    }
  }

  ble_gattc_notify_custom(connectionHandle, transferCharHandle, om);

  // Allow BLE stack to process before potentially sending more
  vTaskDelay(50);
}

void HeartRateService::SendClearResponse(uint16_t connectionHandle, bool success) {
  ClearResponse response {
    .command = static_cast<uint8_t>(Commands::CLEAR_DATA_RESPONSE),
    .status = success ? static_cast<uint8_t>(0x01) : static_cast<uint8_t>(0x00)
  };

  auto* om = ble_hs_mbuf_from_flat(&response, sizeof(response));
  if (om != nullptr) {
    ble_gattc_notify_custom(connectionHandle, transferCharHandle, om);
  }
}

void HeartRateService::SubscribeNotification(uint16_t attributeHandle) {
  if (attributeHandle == transferCharHandle)
    notificationEnabled = true;
}

void HeartRateService::UnsubscribeNotification(uint16_t attributeHandle) {
  if (attributeHandle == transferCharHandle)
    notificationEnabled = false;
}
