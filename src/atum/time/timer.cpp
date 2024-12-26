#include "timer.hpp"

namespace atum {
Timer::Timer(const second_t iAlarmTime) :
    startTime{time()},
    alarmTime{iAlarmTime} {}

void Timer::setAlarmTime(const second_t iAlarmTime) {
  alarmTime = iAlarmTime;
}

void Timer::resetAlarm(const second_t resetTime) {
  startTime = time() - resetTime;
}

bool Timer::goneOff() const {
  return timeElapsed() >= alarmTime;
}

second_t Timer::timeElapsed() const {
  return time() - startTime;
}

Condition Timer::checkGoneOff() const {
  return [=]() { return goneOff(); };
}
} // namespace atum