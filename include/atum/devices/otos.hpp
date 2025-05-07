/**
 * @file otos.hpp
 * @brief Includes the OTOS class.
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "../devices/serialDevice.hpp"
#include "../pose/pose.hpp"
#include "../time/task.hpp"
#include "../time/timer.hpp"

namespace atum {
/**
 * @brief This class encapsulates the logic behind the Sparkfun Optical Tracking
 * Odometry Sensor (OTOS). It mostly acts as a wrapper around the SerialDevice
 * class, with it's own set of commands and responses.
 *
 * The OTOS should be set to receive data relative to the robot, not the field.
 *
 */
class OTOS : public Task {
  TASK_BOILERPLATE(); // Included in all task derivatives for setup.

  public:
  /**
   * @brief Constructs a new OTOS. Offset referring to where the OTOS is
   * relative to the center of the bot.
   *
   * @param port
   * @param offset
   * @param loggerLevel
   */
  OTOS(const std::int8_t port,
       const UnwrappedPose &offset = Pose{},
       const Logger::Level loggerLevel = Logger::Level::Info);

       /**
        * @brief Gets the forward velocity according to the OTOS. 
        * 
        * @return meters_per_second_t 
        */
  meters_per_second_t getVF() const;

  /**
   * @brief Gets the side velocity according to the OTOS.
   * 
   * @return meters_per_second_t 
   */
  meters_per_second_t getVS() const;

  /**
   * @brief Gets the angular velocity according to the OTOS.
   * 
   * @return radians_per_second_t 
   */
  radians_per_second_t getOmega() const;

  bool check();

  private:
  /**
   * @brief The various commands the OTOS can accept.
   * 
   */
  enum Command {
    Initialize,
    Calibrate,
    IsCalibrating,
    Reset,
    SetOffset,
    SetPosition,
    GetPosition,
    GetVelocity,
    Check,
    SelfTest,
    Invalid // ALL NEW COMMANDS SHOULD BE PLACED ABOVE THIS ONE!
  };

  /**
   * @brief The various responses the OTOS can send back.
   * 
   */
  enum Response { Success, Error, Waiting, Unknown };

  /**
   * @brief The format of data from the OTOS. 
   * 
   */
  struct OTOSData {
    float x;
    float y;
    float h;
  };

  /**
   * @brief Time dedicated to allowing the OTOS to finish calibrating. 
   * 
   */
  const second_t calibrationTimeout{1_s};

  std::unique_ptr<SerialDevice> otos;
  Logger logger;
  Timer timer;
  meters_per_second_t vf;
  meters_per_second_t vs;
  radians_per_second_t omega;
};
} // namespace atum