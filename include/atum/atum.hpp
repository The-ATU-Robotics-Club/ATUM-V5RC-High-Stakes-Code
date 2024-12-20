#pragma once

/**
 * @brief The following lists the header files of the ATUM API.
 * This is to make including all of these into specific robot files
 * much easier.
 *
 */
#include "controllers/controller.hpp"
#include "controllers/pid.hpp"
#include "controllers/slewRate.hpp"
#include "controllers/tbh.hpp"
#include "depend/units.h"
#include "devices/adi.hpp"
#include "devices/colorSensor.hpp"
#include "devices/distanceSensor.hpp"
#include "devices/imu.hpp"
#include "devices/lineTracker.hpp"
#include "devices/motor.hpp"
#include "devices/odometer.hpp"
#include "devices/piston.hpp"
#include "devices/potentiometer.hpp"
#include "devices/rotationSensor.hpp"
#include "gui/graph.hpp"
#include "gui/log.hpp"
#include "gui/manager.hpp"
#include "gui/map.hpp"
#include "gui/routines.hpp"
#include "gui/screen.hpp"
#include "pose/odometry.hpp"
#include "pose/tracker.hpp"
#include "systems/drive.hpp"
#include "systems/remote.hpp"
#include "systems/robot.hpp"
#include "systems/stateMachine.hpp"
#include "time/schedule.hpp"
#include "time/task.hpp"
#include "time/time.hpp"
#include "time/timer.hpp"
#include "utility/acceptable.hpp"
#include "utility/logger.hpp"
#include "utility/misc.hpp"
#include "utility/units.hpp"
