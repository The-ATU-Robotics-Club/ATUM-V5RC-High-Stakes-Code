#pragma once

#include "../devices/imu.hpp"
#include "../devices/odometer.hpp"
#include "../time/task.hpp"
#include "../utility/units.hpp"
#include "tracker.hpp"

namespace atum {
class Odometry : public Tracker, public Task {
  public:
  Odometry(std::unique_ptr<Odometer> iForward,
           std::unique_ptr<Odometer> iSide,
           std::unique_ptr<IMU> iImu,
           Logger::Level loggerLevel = Logger::Level::Info);

  virtual Position update() override;

  private:
  TASK_BOILERPLATE();

  Position integratePosition(inch_t dx, inch_t dy, radian_t dh);

  std::unique_ptr<Odometer> forward;
  std::unique_ptr<Odometer> side;
  std::unique_ptr<IMU> imu;
};
} // namespace atum