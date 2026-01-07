#pragma once

#include <cstdint>

namespace Pinetime {
  namespace Controllers {
    class FirmwareValidator {
    public:
      void Validate();
      bool IsValidated() const;

      void Reset();

    private:
      static constexpr uint32_t validBitAdress {0x7BFE8};
      // MCUboot stores the "image_ok" flag as a single byte. Internal flash writes are 32-bit word writes,
      // so we must avoid clearing adjacent bytes in the trailer (they should remain 0xFF).
      static constexpr uint8_t validBitValue {1};
      static constexpr uint32_t validWordValue {0xFFFFFF00u | validBitValue};
    };
  }
}
