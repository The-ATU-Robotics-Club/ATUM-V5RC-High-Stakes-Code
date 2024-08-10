#pragma once

#define PROS_USE_LITERALS

#include "api.h"
#include "atum/atum.hpp"
#include "pros/apix.h"
#include "robots/15in/robot15In.hpp"
#include "robots/24in/robot24In.hpp"
#include "robots/descore/robotDescore.hpp"

#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif
