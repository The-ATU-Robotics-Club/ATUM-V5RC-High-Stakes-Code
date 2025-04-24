#include "serialDevice.hpp"

namespace atum {
SerialDevice::SerialDevice(const int8_t port,
                           const uint8_t iErrorID,
                           const int iTimeout,
                           const int32_t baudrate) :
    serial{std::make_unique<pros::Serial>(port, baudrate)},
    errorID{iErrorID},
    timeout{iTimeout} {
  pros::Device::get_all_devices(pros::DeviceType::serial);
  pros::delay(100);
  serial->flush();
}

Packet SerialDevice::msg(const Packet &command,
                         const int expectedResponseSize) {
  serial->flush();
  write(command);
  const uint32_t startTime{pros::millis()};
  while(serial->get_read_avail() < expectedResponseSize &&
        pros::millis() - startTime < timeout) {
    pros::delay(0); // Allow context switching while idling.
  }
  if(serial->get_read_avail() >= expectedResponseSize) {
    Packet response{read()};
    if(response.correct()) {
      return response;
    }
  }
  return Packet{errorID};
}

void SerialDevice::write(const Packet &command) {
  serial->write_byte(command.id);
  serial->write_byte(command.checksum);
  for(uint8_t byte : command.data) {
    serial->write_byte(byte);
  }
}

Packet SerialDevice::read() const {
  Packet response;
  response.id = serial->read_byte();
  response.checksum = serial->read_byte();
  while(serial->get_read_avail()) {
    response.data.push_back(serial->read_byte());
  }
  return response;
}
} // namespace atum