#pragma once
#define min // workaround: nimble's min/max macros conflict with libstdc++
#define max
#include <host/ble_gap.h>
#undef max
#undef min
#include <atomic>
#include <cstdint>

namespace Pinetime {
  namespace Controllers {
    class MotionController;
    class NimbleController;

    class HeartRateService {
    public:
      HeartRateService(NimbleController& nimble, Controllers::MotionController& motionController);
      void Init();
      int OnDataRequested(uint16_t connectionHandle, uint16_t attributeHandle, ble_gatt_access_ctxt* context);

      void SubscribeNotification(uint16_t attributeHandle);
      void UnsubscribeNotification(uint16_t attributeHandle);

      // Custom service UUID for accelerometer data (generated)
      // Base: ADAF0000-4669-6C65-5472-616E73666572
      static constexpr ble_uuid128_t accelDataServiceUuid {
        .u {.type = BLE_UUID_TYPE_128},
        .value = {0x72, 0x65, 0x66, 0x73, 0x6e, 0x61, 0x72, 0x54, 0x65, 0x6c, 0x69, 0x46, 0x00, 0xAC, 0xAF, 0xAD}};

    private:
      NimbleController& nimble;
      Controllers::MotionController& motionController;

      // Transfer characteristic for command/response
      static constexpr ble_uuid128_t transferCharUuid {
        .u {.type = BLE_UUID_TYPE_128},
        .value = {0x72, 0x65, 0x66, 0x73, 0x6e, 0x61, 0x72, 0x54, 0x65, 0x6c, 0x69, 0x46, 0x01, 0xAC, 0xAF, 0xAD}};

      struct ble_gatt_chr_def characteristicDefinition[2];
      struct ble_gatt_svc_def serviceDefinition[2];

      uint16_t transferCharHandle;
      std::atomic_bool notificationEnabled {false};

      // Command types
      enum class Commands : uint8_t {
        GET_COUNT = 0x01,
        GET_COUNT_RESPONSE = 0x02,
        READ_ENTRIES = 0x10,
        READ_ENTRIES_RESPONSE = 0x11,
        CLEAR_DATA = 0x20,
        CLEAR_DATA_RESPONSE = 0x21,
      };

      // Response structures
      struct __attribute__((packed)) CountResponse {
        uint8_t command;
        uint8_t status;
        uint32_t count;
      };

      struct __attribute__((packed)) EntryData {
        float counts;
        int16_t heartRate;
        uint32_t timestamp;
      };

      struct __attribute__((packed)) ReadEntriesHeader {
        uint8_t command;
        uint8_t status;
        uint32_t startIndex;
        uint32_t totalCount;
        uint8_t entriesInPacket;
        // Followed by EntryData entries
      };

      struct __attribute__((packed)) ReadEntriesRequest {
        uint8_t command;
        uint8_t padding;
        uint32_t startIndex;
        uint8_t count;
      };

      struct __attribute__((packed)) ClearResponse {
        uint8_t command;
        uint8_t status;
      };

      int HandleCommand(uint16_t connectionHandle, os_mbuf* om);
      void SendCountResponse(uint16_t connectionHandle);
      void SendEntries(uint16_t connectionHandle, uint32_t startIndex, uint8_t count);
      void SendClearResponse(uint16_t connectionHandle, bool success);
    };
  }
}
