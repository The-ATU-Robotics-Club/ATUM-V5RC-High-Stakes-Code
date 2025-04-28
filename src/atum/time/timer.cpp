#include "timer.hpp"


namespace atum {
Timer::Timer(const second_t iAlarmTime) :
    startTime{time()},
    alarmTime{iAlarmTime},
    previousTime{time()} {}

void Timer::setAlarm(const second_t iAlarmTime) {
  alarmTime = iAlarmTime;
  setTime();
}

void Timer::setTime(const second_t newTime) {
  const second_t dt{startTime - previousTime};
  startTime = time() - newTime;
  previousTime = startTime - dt;
}

void Timer::start() {
  if(started) {
    return;
  }
  setTime();
  started = true;
}

void Timer::restart() {
  if(!started) {
    return;
  }
  started = false;
  setTime();
}

bool Timer::goneOff() const {
  return timeElapsed() >= alarmTime;
}

second_t Timer::timeElapsed() const {
  return time() - startTime;
}

second_t Timer::getDT() {
  const second_t currentTime{time()};
  const second_t dt{currentTime - previousTime};
  previousTime = currentTime;
  return dt;
}

Condition Timer::checkGoneOff() const {
  return [=]() { return goneOff(); };
}
} // namespace atum