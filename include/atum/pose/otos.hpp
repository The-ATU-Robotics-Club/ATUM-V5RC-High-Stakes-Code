#pragma once

#include "../devices/serialDevice.hpp"
#include "tracker.hpp"

namespace atum {
class OTOS : public Tracker {
  public:
  OTOS(const std::int8_t port,
       const UnwrappedPose &offset = Pose{},
       const Logger::Level loggerLevel = Logger::Level::Info);

  Pose update() override;

  void setPose(const Pose &iPose) override;

  bool check() const;

  private:
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

  enum Response { Success, Error, Waiting, Unknown };

  struct OTOSData {
    float x;
    float y;
    float h;
  };

  const second_t calibrationTimeout{1_s};

  std::unique_ptr<SerialDevice> otos;
  Logger logger;
  Timer timer;
};
} // namespace atum