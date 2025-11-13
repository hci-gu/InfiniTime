#pragma once

#include <cstdint>

namespace Pinetime {
  namespace Controllers {
    namespace MinuteDataProtocol {
      constexpr uint8_t kProtocolVersion = 1;
      constexpr uint8_t kControlOpcodeHandshake = 0x01;
      constexpr uint8_t kControlOpcodeRequestRange = 0x02;
      constexpr uint8_t kControlOpcodeAck = 0x03;
      constexpr uint8_t kControlOpcodeAbort = 0x7f;
      constexpr uint8_t kControlOpcodeHandshakeRsp = 0x81;
      constexpr uint8_t kControlOpcodeStatus = 0x82;

      enum class ErrorCode : uint8_t {
        Ok = 0x00,
        Busy = 0x01,
        InvalidRange = 0x02,
        NothingToSend = 0x03,
        Aborted = 0x04,
        InternalError = 0x7f,
      };

      struct __attribute__((packed)) HandshakeResponse {
        uint8_t opcode = kControlOpcodeHandshakeRsp;
        uint8_t version = kProtocolVersion;
        uint16_t sampleSize = 0;
        uint16_t maxWindow = 0;
        uint8_t flags = 0;
        uint8_t reserved = 0;
      };

      struct __attribute__((packed)) StatusFrame {
        uint8_t opcode = kControlOpcodeStatus;
        uint8_t status = 0;
        uint32_t oldest = 0;
        uint32_t newest = 0;
        uint16_t available = 0;
      };

      struct __attribute__((packed)) DataSamplePayload {
        uint16_t sequence = 0;
        uint32_t epoch = 0;
        int16_t acceleration = 0;
        int16_t heartRate = 0;
        uint8_t flags = 0;
      };

      inline void PackDataSample(uint32_t timestamp,
                                 int16_t acceleration,
                                 int16_t heartRate,
                                 uint8_t flags,
                                 uint16_t sequence,
                                 DataSamplePayload& out) {
        out.sequence = sequence;
        out.epoch = timestamp;
        out.acceleration = acceleration;
        out.heartRate = heartRate;
        out.flags = flags;
      }
    }
  }
}
