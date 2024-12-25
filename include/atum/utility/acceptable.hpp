/**
 * @file acceptable.hpp
 * @brief Includes the Acceptable template class and some helpful aliases.
 * @date 2024-12-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "../time/timer.hpp"
#include "units.hpp"
#include <limits>

namespace atum {
/**
 * @brief This class template produces encapsulates several conditions
 * for the results of an action being considered acceptable, including a
 * timeout (past which the action is considered acceptable), a max error
 * (below which, the action is considered acceptable), a max derivative of
 * error (below which, assuming error is fine, the action is considered
 * acceptable) in order to account for momentum, and a minimum time to meet
 * those conditions for.
 *
 * @tparam Unit
 * @tparam UnitsPerSecond
 */
template <typename Unit>
class Acceptable {
  public:
  // Figure out the corresponding types for the derivative of Unit. 
  using UnitsPerSecond = decltype(Unit{1} / 1_s);

  /**
   * @brief Constructs a new acceptable object, checking for the parameters
   * given to see if an action is complete, including timeout (past which the
   * action is considered acceptable), a max error (below which, the action is
   * considered acceptable), a max derivative of error (below which, assuming
   * error is fine, the action is considered acceptable) in order to account for
   * momentum, and a minimum time to meet those conditions for.
   *
   * @param iTimeout
   * @param iMaxError
   * @param iMaxDeriv
   * @param minTime
   * @param loggerLevel
   */
  Acceptable(const second_t iTimeout,
             const Unit &iMaxError = Unit{0.0},
             const UnitsPerSecond &iMaxDeriv = UnitsPerSecond{std::numeric_limits<double>::max()},
             const second_t minTime = 0_s,
             const Logger::Level loggerLevel = Logger::Level::Info) :
      timeout{iTimeout},
      maxError{iMaxError},
      maxDeriv{iMaxDeriv},
      minTimer{minTime},
      logger{loggerLevel} {
    logger.debug("Acceptable checker has been constructed!");
  }

  /**
   * @brief Checks if under the current conditions the results can be accepted.
   *
   * @param state
   * @param reference
   * @return true
   * @return false
   */
  bool canAccept(const Unit &state, const Unit &reference) {
    return canAccept(reference - state);
  }

  /**
   * @brief Checks if under the current conditions the results can be accepted.
   *
   * @param error
   * @return true
   * @return false
   */
  bool canAccept(const Unit &error) {
    if(!timeoutTimer) {
      timeoutTimer = Timer{timeout};
    }
    const second_t currentTime{time()};
    const UnitsPerSecond deriv{(error - prevError) / (currentTime - prevTime)};
    accepted = abs(error) <= maxError;
    accepted = accepted && abs(deriv) <= maxDeriv;
    if(!accepted) {
      minTimer.resetAlarm();
    }
    accepted = accepted && minTimer.goneOff();
    // If given timeout has been passed, accepted should always be true.
    accepted = accepted || timeoutTimer.value().goneOff();
    prevError = error;
    prevTime = currentTime;
    return canAccept(); // Use canAccept() logging purposes.
  }

  /**
   * @brief Returns if the results can be accepted based on the last check.
   * Also performs some logging.
   *
   * @return true
   * @return false
   */
  bool canAccept() {
    if(accepted) {
      logger.debug("Acceptable object can accept these results.");
    }
    return accepted;
  }

  private:
  bool accepted{false};
  const second_t timeout;
  const Unit maxError;
  const UnitsPerSecond maxDeriv;
  Timer minTimer;
  Logger logger;
  std::optional<Timer> timeoutTimer;
  Unit prevError{0};
  second_t prevTime{0_s};
};

/**
 * @brief These aliases are provided for ease of use, since they
 * are particularly common.
 *
 */
using AcceptableDistance = Acceptable<inch_t>;
using AcceptableAngle = Acceptable<degree_t>;
} // namespace atum