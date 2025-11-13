#include <cstdint>
#include <limits>

#include "components/ble/MinuteDataProtocol.h"

using namespace Pinetime::Controllers::MinuteDataProtocol;

namespace {
  void TestPackDataSample() {
    DataSamplePayload payload {};
    constexpr uint32_t timestamp = 1670000000;
    constexpr int16_t accel = -1234;
    constexpr int16_t hr = 77;
    constexpr uint8_t flags = 0x5a;
    constexpr uint16_t seq = 42;
    PackDataSample(timestamp, accel, hr, flags, seq, payload);
    if (payload.sequence != seq || payload.epoch != timestamp || payload.acceleration != accel ||
        payload.heartRate != hr || payload.flags != flags) {
      throw 1;
    }
  }
}

int main() {
  static_assert(sizeof(DataSamplePayload) == 11, "Unexpected data payload size");
  static_assert(sizeof(HandshakeResponse) == 8, "Unexpected handshake payload size");
  static_assert(sizeof(StatusFrame) == 12, "Unexpected status payload size");

  try {
    TestPackDataSample();
  } catch (...) {
    return 1;
  }
  return 0;
}
