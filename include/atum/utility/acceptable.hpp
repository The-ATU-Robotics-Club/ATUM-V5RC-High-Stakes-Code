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
 * @tparam U
 * @tparam dU
 */
template <typename U = double, typename dU = double>
class Acceptable {
  public:
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
             const U &iMaxError = U{0.0},
             const dU &iMaxDeriv = dU{std::numeric_limits<double>::max()},
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
  bool canAccept(const U &state, const U &reference) {
    return canAccept(reference - state);
  }

  /**
   * @brief Checks if under the current conditions the results can be accepted.
   *
   * @param error
   * @return true
   * @return false
   */
  bool canAccept(const U &error) {
    if(!timeoutTimer) {
      timeoutTimer = Timer{timeout};
    }
    const second_t currentTime{time()};
    const dU deriv{(error - prevError) / (currentTime - prevTime)};
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
  const U maxError;
  const dU maxDeriv;
  Logger logger;
  Timer minTimer;
  std::optional<Timer> timeoutTimer;
  U prevError{0};
  second_t prevTime{0_s};
};

/**
 * @brief These aliases are provided for ease of use, since they
 * are particularly common.
 *
 */
using AcceptableDistance = Acceptable<inch_t, feet_per_second_t>;
using AcceptableAngle = Acceptable<degree_t, degrees_per_second_t>;
} // namespace atum