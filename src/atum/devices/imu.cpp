#include "imu.hpp"

namespace atum {
IMU::IMU(std::vector<std::uint8_t> ports,
         const bool iReversed,
         Logger::LoggerLevel loggerLevel) :
    reversed{iReversed}, logger{loggerLevel} {
  for(std::uint8_t port : ports) {
    imus.push_back(std::make_unique<pros::IMU>(port));
    if(pros::c::registry_get_plugged_type(port - 1) !=
       pros::c::v5_device_e_t::E_DEVICE_IMU) {
      logger.error("IMU at port " + std::to_string(port) +
                   " could not be initialized!");
      imus.erase(imus.end());
    }
  }
  for(auto &imu : imus) imu->set_data_rate(5);
  logger.info("IMU is constructed!");
  while(imus.size() && imus.back()->is_calibrating()) pros::delay(10);
  logger.info("IMU is calibrated!");
}

void IMU::setHeading(degree_t heading) {
  if(reversed) heading *= -1;
  for(auto &imu : imus) imu->set_rotation(getValueAs<degree_t>(heading));
  previous = heading;
}

degree_t IMU::getHeading() {
  std::vector<degree_t> readings;
  for(auto &imu : imus) readings.push_back(degree_t{imu->get_rotation()});
  degree_t heading{average(readings)};
  if(reversed) heading *= -1;
  logger.debug("IMU Rotation: " + to_string(heading));
  return heading;
}

degree_t IMU::getTraveled() {
  const degree_t current{getHeading()};
  const degree_t dh{current - previous};
  previous = current;
  return dh;
}
} // namespace atum