/**
 * @file serialDevice.hpp
 * @brief Includes the SerialDevice class.
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "../../pros/serial.hpp"
#include "packet.hpp"

namespace atum {
class SerialDevice {
  public:
  /**
   * @brief Constructs a new SerialDevice object.
   *
   * Error ID will be used in situations where the parity byte is incorrect or
   * messaging times out.
   *
   * The timeout refers to how long to wait for a response after messaging the
   * device.
   *
   * Baudrate refers to the rate of data transfer. Some devices can not support
   * an especially high baudrate. In general, higher rates are less consistent
   * and stable than lower rates. The default (115200) is the practical maximum
   * for the Arduino Nano.
   *
   * @param port
   * @param iErrorID
   * @param iTimeout
   * @param baudrate
   */
  SerialDevice(const int8_t port,
               const uint8_t iErrorID,
               const int iTimeout = 5,
               const int32_t baudrate = 115200);

  /**
   * @brief Send a message and get a response.
   *
   * The default for expected response size is two, since most of the time you
   * just want the ID and parity byte. Occasionally, some additional data will
   * be passed along as well.
   *
   * @param command
   * @param expectedResponseSize
   * @return Packet
   */
  Packet msg(const Packet &command, const int expectedResponseSize = 2);

  private:
  void write(const Packet &command);

  Packet read() const;

  std::unique_ptr<pros::Serial> serial;
  const uint8_t errorID;
  const int timeout;
};
} // namespace atum