#pragma once

#define PROS_USE_LITERALS

#include "api.h"
#include "atum/atum.hpp"
#include "pros/apix.h"
#include "robots/robot15A/robot15A.hpp"
#include "robots/robotPrototype/robotPrototype.hpp"

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