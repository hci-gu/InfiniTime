#include "components/ble/MinuteDataService.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <new>

#include <host/ble_hs.h>
#include <nimble/nimble_port.h>
#include <nrf_log.h>

#include "components/ble/BleController.h"
#include "components/ble/NimbleController.h"
#include "components/datetime/DateTimeController.h"
#include "utility/Span.h"

using namespace Pinetime::Controllers;

namespace {
  // 0004yyxx-78fc-48fe-8e23-433b3a1942d0
  constexpr ble_uuid128_t CharUuid(uint8_t x, uint8_t y) {
    return ble_uuid128_t {.u = {.type = BLE_UUID_TYPE_128},
                          .value = {0xd0, 0x42, 0x19, 0x3a, 0x3b, 0x43, 0x23, 0x8e, 0xfe, 0x48, 0xfc, 0x78, x, y, 0x04, 0x00}};
  }

  constexpr ble_uuid128_t ServiceUuidValue() {
    return CharUuid(0x00, 0x00);
  }

  constexpr ble_uuid128_t controlCharUuid {CharUuid(0x01, 0x00)};
  constexpr ble_uuid128_t statusCharUuid {CharUuid(0x02, 0x00)};
  constexpr ble_uuid128_t dataCharUuid {CharUuid(0x03, 0x00)};

  uint16_t ReadLe16(const uint8_t* ptr) {
    return static_cast<uint16_t>(ptr[0]) | (static_cast<uint16_t>(ptr[1]) << 8);
  }

  uint32_t ReadLe32(const uint8_t* ptr) {
    return static_cast<uint32_t>(ptr[0]) | (static_cast<uint32_t>(ptr[1]) << 8) |
           (static_cast<uint32_t>(ptr[2]) << 16) | (static_cast<uint32_t>(ptr[3]) << 24);
  }
}

const ble_uuid128_t& MinuteDataService::ServiceUuid() {
  static constexpr ble_uuid128_t uuid = ServiceUuidValue();
  return uuid;
}

MinuteDataService::MinuteDataService(NimbleController& nimble,
                                     MotionController& motionController,
                                     DateTime& dateTimeController,
                                     Ble& bleController)
  : nimble {nimble},
    motionController {motionController},
    dateTimeController {dateTimeController},
    bleController {bleController},
    characteristicDefinition {{.uuid = &controlCharUuid.u,
                               .access_cb = ControlCallback,
                               .arg = this,
                               .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_INDICATE,
                               .val_handle = &controlHandle},
                              {.uuid = &statusCharUuid.u,
                               .access_cb = StatusCallback,
                               .arg = this,
                               .flags = BLE_GATT_CHR_F_READ,
                               .val_handle = &statusHandle},
                              {.uuid = &dataCharUuid.u,
                               .access_cb = nullptr,
                               .arg = nullptr,
                               .flags = BLE_GATT_CHR_F_NOTIFY,
                               .val_handle = &dataHandle},
                              {0}},
    serviceDefinition {{.type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &ServiceUuid().u, .characteristics = characteristicDefinition},
                       {0}} {
}

void MinuteDataService::Init() {
  if (!calloutInitialized) {
    auto* eventQueue = nimble_port_get_dflt_eventq();
    if (eventQueue != nullptr) {
      ble_npl_callout_init(&sendCallout, eventQueue, SendCalloutCallback, this);
      calloutInitialized = true;
    }
  }

  int res = ble_gatts_count_cfg(serviceDefinition);
  ASSERT(res == 0);
  res = ble_gatts_add_svcs(serviceDefinition);
  ASSERT(res == 0);
}

bool MinuteDataService::ShouldAdvertise() const {
  return motionController.LoggedMinuteCount() > 0;
}

void MinuteDataService::OnConnect(uint16_t connHandle) {
  connectionHandle = connHandle;
  peerMtu = 0;
}

void MinuteDataService::OnDisconnect() {
  connectionHandle = BLE_HS_CONN_HANDLE_NONE;
  dataNotificationEnabled = false;
  controlIndicationEnabled = false;
  HandleAbort();
}

int MinuteDataService::ControlCallback(uint16_t conn_handle,
                                       uint16_t attr_handle,
                                       ble_gatt_access_ctxt* ctxt,
                                       void* arg) {
  auto* service = static_cast<MinuteDataService*>(arg);
  return service->OnControlAccess(conn_handle, attr_handle, ctxt);
}

int MinuteDataService::StatusCallback(uint16_t conn_handle,
                                      uint16_t attr_handle,
                                      ble_gatt_access_ctxt* ctxt,
                                      void* arg) {
  auto* service = static_cast<MinuteDataService*>(arg);
  (void)conn_handle;
  (void)attr_handle;
  return service->OnStatusAccess(ctxt);
}

int MinuteDataService::OnControlAccess(uint16_t conn_handle,
                                       uint16_t attr_handle,
                                       ble_gatt_access_ctxt* ctxt) {
  if (attr_handle != controlHandle || ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }

  if (ctxt->om == nullptr || OS_MBUF_PKTLEN(ctxt->om) == 0) {
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }

  uint8_t buffer[20];
  size_t len = std::min(static_cast<size_t>(OS_MBUF_PKTLEN(ctxt->om)), sizeof(buffer));
  os_mbuf_copydata(ctxt->om, 0, len, buffer);

  const uint8_t opcode = buffer[0];
  switch (opcode) {
    case MinuteDataProtocol::kControlOpcodeHandshake:
      HandleHandshake(conn_handle, buffer, len);
      break;
    case MinuteDataProtocol::kControlOpcodeRequestRange:
      HandleRangeRequest(conn_handle, buffer, len);
      break;
    case MinuteDataProtocol::kControlOpcodeAck:
      HandleAck(buffer, len);
      break;
    case MinuteDataProtocol::kControlOpcodeAbort:
      HandleAbort();
      break;
    default:
      SendStatusFrame(MinuteDataProtocol::ErrorCode::InvalidRange);
      break;
  }
  return 0;
}

int MinuteDataService::OnStatusAccess(ble_gatt_access_ctxt* ctxt) {
  MinuteDataProtocol::StatusFrame status;
  status.status = static_cast<uint8_t>(MinuteDataProtocol::ErrorCode::Ok);
  auto range = motionController.OldestNewest();
  if (range.count == 0) {
    auto now = NowEpochSeconds();
    status.oldest = now;
    status.newest = now;
    status.available = 0;
  } else {
    status.oldest = range.oldestEpoch;
    status.newest = range.newestEpoch;
    status.available = static_cast<uint16_t>(std::min<size_t>(range.count, std::numeric_limits<uint16_t>::max()));
  }

  int res = os_mbuf_append(ctxt->om, &status, sizeof(status));
  return (res == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

void MinuteDataService::SubscribeNotification(uint16_t attributeHandle, bool notify, bool indicate) {
  if (attributeHandle == dataHandle && notify) {
    dataNotificationEnabled = true;
    if (transferState == TransferState::Streaming) {
      ScheduleNextChunk();
    }
  } else if (attributeHandle == controlHandle && indicate) {
    controlIndicationEnabled = true;
  }
}

void MinuteDataService::UnsubscribeNotification(uint16_t attributeHandle, bool notify, bool indicate) {
  if (attributeHandle == dataHandle && notify) {
    dataNotificationEnabled = false;
  } else if (attributeHandle == controlHandle && indicate) {
    controlIndicationEnabled = false;
  }
}

void MinuteDataService::HandleHandshake(uint16_t conn_handle, const uint8_t* data, size_t len) {
  if (len < 4) {
    SendStatusFrame(MinuteDataProtocol::ErrorCode::InvalidRange);
    return;
  }

  peerMtu = ReadLe16(&data[2]);
  MinuteDataProtocol::HandshakeResponse response;
  response.sampleSize = sizeof(MinuteDataProtocol::DataSamplePayload);
  response.maxWindow = MotionController::MinuteLogCapacity;
  response.flags = 0;
  SendControlResponse(reinterpret_cast<const uint8_t*>(&response), sizeof(response));

  uint16_t actualMtu = NegotiatedMtu(conn_handle);
  NRF_LOG_INFO("Minute data handshake, peer mtu=%d negotiated=%d", peerMtu, actualMtu);
}

void MinuteDataService::HandleRangeRequest(uint16_t conn_handle, const uint8_t* data, size_t len) {
  if (len < 7) {
    SendStatusFrame(MinuteDataProtocol::ErrorCode::InvalidRange);
    return;
  }

  if (transferState == TransferState::Streaming) {
    SendStatusFrame(MinuteDataProtocol::ErrorCode::Busy);
    return;
  }

  uint32_t startEpoch = ReadLe32(&data[1]);
  uint16_t requested = ReadLe16(&data[5]);
  size_t requestCount = std::min(static_cast<size_t>(requested), static_cast<size_t>(MotionController::MinuteLogCapacity));
  if (requestCount == 0) {
    SendStatusFrame(MinuteDataProtocol::ErrorCode::InvalidRange);
    return;
  }

  if (!EnsureTransferBuffer(requestCount)) {
    SendStatusFrame(MinuteDataProtocol::ErrorCode::InternalError);
    return;
  }

  auto span = gsl::span<MotionController::MinuteSample> {transferBuffer.get(), requestCount};
  size_t copied = motionController.CopyMinutes(startEpoch, requestCount, span);
  if (copied == 0) {
    SendStatusFrame(MinuteDataProtocol::ErrorCode::NothingToSend);
    return;
  }
  NRF_LOG_INFO("Minute data range request start=%lu count=%u available=%u",
               static_cast<unsigned long>(startEpoch),
               static_cast<unsigned>(requestCount),
               static_cast<unsigned>(copied));

  currentTransfer.startEpoch = startEpoch;
  currentTransfer.requestedCount = requestCount;
  currentTransfer.bufferedSamples = copied;
  currentTransfer.nextSample = 0;
  currentTransfer.acknowledgedIndex = 0;
  nextSequence = 1;
  lastSentSequence = 0;
  transferState = TransferState::Streaming;
  connectionHandle = conn_handle;
  ScheduleNextChunk();
}

void MinuteDataService::HandleAck(const uint8_t* data, size_t len) {
  if (len < 3) {
    return;
  }

  if (lastSentSequence == 0 || transferBuffer == nullptr) {
    return;
  }

  uint16_t ackSeq = ReadLe16(&data[1]);
  if (ackSeq == 0) {
    return;
  }

  if (ackSeq > lastSentSequence) {
    ackSeq = lastSentSequence;
  }

  if (ackSeq <= currentTransfer.acknowledgedIndex) {
    return;
  }
  size_t ackIndex = static_cast<size_t>(ackSeq);
  if (ackIndex > currentTransfer.bufferedSamples) {
    ackIndex = currentTransfer.bufferedSamples;
  }

  currentTransfer.acknowledgedIndex = ackIndex;
  if (ackIndex == 0) {
    return;
  }

  const auto& sample = transferBuffer[ackIndex - 1];
  motionController.MarkMinutesTransferred(sample.timestamp);

  if (ackIndex >= currentTransfer.bufferedSamples) {
    lastSentSequence = 0;
    ReleaseTransferBuffer();
  }
}

void MinuteDataService::HandleAbort() {
  if (transferState == TransferState::Idle) {
    ReleaseTransferBuffer();
    return;
  }

  if (calloutInitialized) {
    ble_npl_callout_stop(&sendCallout);
  }
  NRF_LOG_INFO("Minute data transfer aborted");
  transferState = TransferState::Idle;
  nextSequence = 1;
  lastSentSequence = 0;
  ReleaseTransferBuffer();
}

void MinuteDataService::SendControlResponse(const uint8_t* data, size_t len) {
  if (!controlIndicationEnabled) {
    return;
  }

  if (connectionHandle == BLE_HS_CONN_HANDLE_NONE) {
    return;
  }

  auto* om = ble_hs_mbuf_from_flat(data, len);
  if (om == nullptr) {
    return;
  }

  ble_gattc_indicate_custom(connectionHandle, controlHandle, om);
}

void MinuteDataService::SendStatusFrame(MinuteDataProtocol::ErrorCode code) {
  MinuteDataProtocol::StatusFrame status;
  status.status = static_cast<uint8_t>(code);
  auto range = motionController.OldestNewest();
  if (range.count == 0) {
    auto now = NowEpochSeconds();
    status.oldest = now;
    status.newest = now;
    status.available = 0;
  } else {
    status.oldest = range.oldestEpoch;
    status.newest = range.newestEpoch;
    status.available = static_cast<uint16_t>(std::min<size_t>(range.count, std::numeric_limits<uint16_t>::max()));
  }
  SendControlResponse(reinterpret_cast<const uint8_t*>(&status), sizeof(status));
}

bool MinuteDataService::CanStream() const {
  return transferState == TransferState::Streaming && dataNotificationEnabled && connectionHandle != BLE_HS_CONN_HANDLE_NONE &&
         bleController.IsConnected();
}

void MinuteDataService::ScheduleNextChunk() {
  if (!CanStream()) {
    return;
  }

  if (!calloutInitialized) {
    return;
  }

  if (ble_npl_callout_is_active(&sendCallout)) {
    return;
  }

  ble_npl_callout_reset(&sendCallout, ble_npl_time_ms_to_ticks32(20));
}

void MinuteDataService::SendCalloutCallback(ble_npl_event* event) {
  auto* service = static_cast<MinuteDataService*>(ble_npl_event_get_arg(event));
  service->SendNextChunk();
}

uint16_t MinuteDataService::NegotiatedMtu(uint16_t conn_handle) const {
  uint16_t mtu = ble_att_mtu(conn_handle);
  if (mtu == 0) {
    mtu = BLE_ATT_MTU_DFLT;
  }
  if (peerMtu != 0) {
    mtu = std::min(mtu, peerMtu);
  }
  return mtu;
}

uint32_t MinuteDataService::NowEpochSeconds() const {
  auto now = dateTimeController.CurrentDateTime();
  auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
  auto epoch = std::chrono::duration_cast<std::chrono::seconds>(seconds.time_since_epoch());
  return static_cast<uint32_t>(epoch.count());
}

void MinuteDataService::SendNextChunk() {
  if (!CanStream()) {
    return;
  }

  if (transferBuffer == nullptr) {
    transferState = TransferState::Idle;
    SendStatusFrame(MinuteDataProtocol::ErrorCode::InternalError);
    return;
  }

  uint16_t mtu = NegotiatedMtu(connectionHandle);
  size_t maxPayload = (mtu > 3) ? static_cast<size_t>(mtu - 3) : notifyBuffer.size();
  maxPayload = std::min(maxPayload, notifyBuffer.size());

  size_t offset = 0;
  size_t samplesSent = 0;
  while (currentTransfer.nextSample < currentTransfer.bufferedSamples) {
    const auto& sample = transferBuffer[currentTransfer.nextSample];
    MinuteDataProtocol::DataSamplePayload payload;
    MinuteDataProtocol::PackDataSample(
      sample.timestamp, sample.accelerationAverage, sample.heartRateAverage, sample.flags, nextSequence, payload);

    if (offset + sizeof(payload) > maxPayload) {
      break;
    }

    std::memcpy(&notifyBuffer[offset], &payload, sizeof(payload));
    offset += sizeof(payload);
    currentTransfer.nextSample++;
    samplesSent++;
    lastSentSequence = nextSequence;
    nextSequence++;
  }

  if (samplesSent == 0) {
    transferState = TransferState::Idle;
    SendStatusFrame(MinuteDataProtocol::ErrorCode::Ok);
    return;
  }

  auto* om = ble_hs_mbuf_from_flat(notifyBuffer.data(), offset);
  if (om == nullptr) {
    ScheduleNextChunk();
    return;
  }

  int rc = ble_gattc_notify_custom(connectionHandle, dataHandle, om);
  if (rc != 0) {
    transferState = TransferState::Idle;
    SendStatusFrame(MinuteDataProtocol::ErrorCode::InternalError);
    return;
  }

  if (currentTransfer.nextSample < currentTransfer.bufferedSamples) {
    ScheduleNextChunk();
  } else {
    NRF_LOG_INFO("Minute data transfer complete; samples=%u", static_cast<unsigned>(currentTransfer.bufferedSamples));
    transferState = TransferState::Idle;
    SendStatusFrame(MinuteDataProtocol::ErrorCode::Ok);
  }
}

bool MinuteDataService::EnsureTransferBuffer(size_t requiredSamples) {
  if (requiredSamples == 0) {
    return false;
  }

  if (transferBuffer != nullptr && transferBufferCapacity >= requiredSamples) {
    return true;
  }

  auto* buffer = new (std::nothrow) MotionController::MinuteSample[requiredSamples];
  if (buffer == nullptr) {
    return false;
  }

  transferBuffer.reset(buffer);
  transferBufferCapacity = requiredSamples;
  return true;
}

void MinuteDataService::ReleaseTransferBuffer() {
  transferBuffer.reset();
  transferBufferCapacity = 0;
  currentTransfer = {};
}
