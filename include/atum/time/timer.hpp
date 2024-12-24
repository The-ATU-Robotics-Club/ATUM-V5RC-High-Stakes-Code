/**
 * @file timer.hpp
 * @brief Includes the Timer class.
 * @date 2024-12-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "time.hpp"

namespace atum {
class Timer {
  public:
  /**
   * @brief Constructs a new timer with an alarm time at which
   * it will be considered to have gone off. The default of 0 s
   * will make it so that the alarm is never considered to have
   * gone off.
   *
   * @param iAlarmTime
   */
  explicit Timer(const second_t iAlarmTime = 0_s);

  /**
   * @brief Sets the alarm time, the time it will take for the alarm
   * to go off after being reset.
   *
   * @param iAlarmTime
   */
  void setAlarmTime(const second_t iAlarmTime);

  /**
   * @brief Resets the timer to a specified time (so that an additional alarm
   * time must pass before the alarm has gone off); default is 0s.
   *
   * @param resetTime
   */
  void resetAlarm(const second_t resetTime = 0_s);

  /**
   * @brief Says if the alarm time has elapsed since last reset is greater
   * than the alarm time (or if the alarm time is zero seconds).
   *
   * @return true
   * @return false
   */
  bool goneOff() const;

  /**
   * @brief Gets the time elapsed since the alarm was created.
   *
   * @return second_t
   */
  second_t timeElapsed() const;

  /**
   * @brief A condition to be used for scheduling or waiting. Returns a function
   * that, when called, returns true if the alarm has gone off.
   *
   * @param desired
   * @return Condition
   */
  Condition checkGoneOff() const;

  private:
  second_t startTime;
  second_t alarmTime;
};
} // namespace atum