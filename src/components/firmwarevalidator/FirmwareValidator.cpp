#include "components/firmwarevalidator/FirmwareValidator.h"

#include <hal/nrf_rtc.h>
#include "drivers/InternalFlash.h"

using namespace Pinetime::Controllers;

bool FirmwareValidator::IsValidated() const {
  auto* imageOkPtr = reinterpret_cast<uint32_t*>(validBitAdress);
  return ((*imageOkPtr) & 0xFFu) == validBitValue;
}

void FirmwareValidator::Validate() {
  if (!IsValidated())
    Pinetime::Drivers::InternalFlash::WriteWord(validBitAdress, validWordValue);
}

void FirmwareValidator::Reset() {
  NVIC_SystemReset();
}
