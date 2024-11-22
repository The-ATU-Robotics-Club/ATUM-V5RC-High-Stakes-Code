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
           Logger::LoggerLevel iLoggerLevel = Logger::LoggerLevel::Info);

  virtual Position update() override;

  protected:
  std::tuple<inch_t, inch_t, radian_t> getDeltas();

  std::pair<inch_t, inch_t>
      getRelativeDeltas(const inch_t dl, const inch_t dr, const radian_t dh);

  Position integratePosition(inch_t dx, inch_t dy, radian_t dh);

  std::unique_ptr<Odometer> forward;
  std::unique_ptr<Odometer> side;
  std::unique_ptr<IMU> imu;

  private:
  void backgroundTask() override;
};
} // namespace atum