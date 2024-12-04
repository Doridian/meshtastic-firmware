#pragma once

#define ARCH_PORTDUINO 1

//
// set HW_VENDOR
//

#define HW_VENDOR meshtastic_HardwareModel_PORTDUINO

#ifndef HAS_WIFI
#define HAS_WIFI 1
#endif
#ifndef HAS_RTC
#define HAS_RTC 1
#endif
#ifndef HAS_TELEMETRY
#define HAS_TELEMETRY 1
#endif

#define LR11X0_DIO3_TCXO_VOLTAGE 3.0
