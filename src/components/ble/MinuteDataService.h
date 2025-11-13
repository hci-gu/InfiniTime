#pragma once

#include <array>
#include <cstdint>
#include <memory>

#define min
#define max
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <nimble/nimble_npl.h>
#include <os/os.h>
#undef max
#undef min

#include "components/ble/MinuteDataProtocol.h"
#include "components/motion/MotionController.h"

namespace Pinetime {
  namespace Controllers {
    class NimbleController;
    class MotionController;
    class DateTime;
    class Ble;

    class MinuteDataService {
    public:
      MinuteDataService(NimbleController& nimble,
                        MotionController& motionController,
                        DateTime& dateTimeController,
                        Ble& bleController);

      void Init();
      int OnControlAccess(uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt* ctxt);
      int OnStatusAccess(ble_gatt_access_ctxt* ctxt);
      void SubscribeNotification(uint16_t attributeHandle, bool notify, bool indicate);
      void UnsubscribeNotification(uint16_t attributeHandle, bool notify, bool indicate);
      void OnConnect(uint16_t connHandle);
      void OnDisconnect();
      bool ShouldAdvertise() const;
      static const ble_uuid128_t& ServiceUuid();

    private:
      enum class TransferState { Idle, Streaming };

      struct RangeRequest {
        uint32_t startEpoch = 0;
        size_t requestedCount = 0;
        size_t bufferedSamples = 0;
        size_t nextSample = 0;
        size_t acknowledgedIndex = 0;
      };

      void HandleHandshake(uint16_t conn_handle, const uint8_t* data, size_t len);
      void HandleRangeRequest(uint16_t conn_handle, const uint8_t* data, size_t len);
      void HandleAck(const uint8_t* data, size_t len);
      void HandleAbort();
      void SendControlResponse(const uint8_t* data, size_t len);
      void SendStatusFrame(MinuteDataProtocol::ErrorCode code);
      void ScheduleNextChunk();
      void SendNextChunk();
      bool CanStream() const;
      uint16_t NegotiatedMtu(uint16_t conn_handle) const;
      uint32_t NowEpochSeconds() const;
      bool EnsureTransferBuffer(size_t requiredSamples);
      void ReleaseTransferBuffer();

      static int ControlCallback(uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt* ctxt, void* arg);
      static int StatusCallback(uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt* ctxt, void* arg);
      static void SendCalloutCallback(ble_npl_event* event);

      NimbleController& nimble;
      MotionController& motionController;
      DateTime& dateTimeController;
      Ble& bleController;

      struct ble_gatt_chr_def characteristicDefinition[4];
      struct ble_gatt_svc_def serviceDefinition[2];

      uint16_t controlHandle = 0;
      uint16_t statusHandle = 0;
      uint16_t dataHandle = 0;

      bool controlIndicationEnabled = false;
      bool dataNotificationEnabled = false;

      TransferState transferState = TransferState::Idle;
      RangeRequest currentTransfer;
      uint16_t nextSequence = 1;
      uint16_t peerMtu = 0;
      uint16_t lastSentSequence = 0;
      uint16_t connectionHandle = BLE_HS_CONN_HANDLE_NONE;

      ble_npl_callout sendCallout {};
      bool calloutInitialized = false;
      std::unique_ptr<MotionController::MinuteSample[]> transferBuffer;
      size_t transferBufferCapacity = 0;
      std::array<uint8_t, 244> notifyBuffer {};
    };
  }
}
