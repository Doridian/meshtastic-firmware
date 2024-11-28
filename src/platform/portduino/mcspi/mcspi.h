#pragma once

#include <HardwareSPI.h>

namespace arduino {
class MCSPICls : public HardwareSPI {
public:
  MCSPICls();
  void begin(uint32_t freq = 2000000) override;
  void begin(const char *name, uint32_t freq = 2000000) override;
  int gpioRead(const uint8_t pin);
};
} // namespace arduino

extern MCSPICls MCSPI;
