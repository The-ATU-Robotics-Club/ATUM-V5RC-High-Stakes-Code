#include "imu.hpp"

namespace atum {
IMU::IMU(std::vector<std::uint8_t> ports,
         const bool iReversed,
         Logger::Level loggerLevel) :
    reversed{iReversed}, logger{loggerLevel} {
  for(const std::uint8_t port : ports) {
    pros::v5::Device device{port};
    if(device.get_plugged_type() == pros::DeviceType::imu) {
      imus.push_back(std::make_unique<pros::IMU>(port));
      logger.debug("IMU found on port " + std::to_string(device.get_port()) +
                   ".");
    } else {
      logger.warn("IMU at port " + std::to_string(port) +
                  " could not be initialized!");
    }
  }
  if(!imus.size()) {
    logger.error("No IMUs found!");
  }
  initializeIMUs();
}

IMU::IMU(const std::size_t minimumAmount,
         const bool iReversed,
         Logger::Level loggerLevel) :
    reversed{iReversed}, logger{loggerLevel} {
  const auto rawIMUs{pros::Distance::get_all_devices()};
  if(!rawIMUs.size()) {
    logger.error("No IMUs found!");
    return;
  } else if(imus.size() < minimumAmount) {
    logger.warn("Number of IMUs found lower than minimum!");
  }
  for(auto imu : rawIMUs) {
    imus.push_back(std::make_unique<pros::IMU>(imu.get_port()));
    logger.debug("IMU found on port " + std::to_string(imu.get_port()) + ".");
  }
  initializeIMUs();
}

void IMU::setHeading(degree_t heading) {
  if(reversed) {
    heading *= -1;
  }
  for(auto &imu : imus) {
    imu->set_rotation(getValueAs<degree_t>(heading));
  }
  previous = heading;
  logger.debug("IMU heading set to " + to_string(heading) + ".");
}

degree_t IMU::getHeading() {
  std::vector<degree_t> readings;
  for(auto &imu : imus) readings.push_back(degree_t{imu->get_rotation()});
  degree_t heading{average(readings)};
  if(reversed) heading *= -1;
  logger.debug("IMU is reading " + to_string(heading) + ".");
  return heading;
}

degree_t IMU::getTraveled() {
  const degree_t current{getHeading()};
  const degree_t dh{current - previous};
  previous = current;
  logger.debug("IMU has traveled " + to_string(dh) + " since last called.");
  return dh;
}

void IMU::initializeIMUs() {
  for(const auto &imu : imus) {
    imu->set_data_rate(5); // Increase refresh rate of IMUs.
  }
  logger.info("IMU is constructed!");
  while(imus.size() && imus.back()->is_calibrating()) {
    wait(10_ms);
  }
  logger.info("IMU is calibrated!");
}
} // namespace atum