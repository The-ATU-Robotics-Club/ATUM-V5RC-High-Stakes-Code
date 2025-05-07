#include "otos.hpp"

namespace atum {
OTOS::OTOS(const std::int8_t port,
           const UnwrappedPose &offset,
           const Logger::Level loggerLevel) :
    Task(this, loggerLevel),
    otos{std::make_unique<SerialDevice>(port, OTOS::Response::Error)},
    logger{loggerLevel} {
  otos->msg({Command::Initialize});
  wait(500_ms);
  otos->msg({Command::Reset});
  PiecedData<OTOSData> offsetPieced;
  offsetPieced.value = {offset.x, offset.y, offset.h};
  otos->msg(Packet{Command::SetOffset,
                   std::vector<uint8_t>(std::begin(offsetPieced.bytes),
                                        std::end(offsetPieced.bytes))},
            2);
  otos->msg({Command::Calibrate});
  Timer timeout{calibrationTimeout};
  while(!timeout.goneOff() &&
        otos->msg({Command::IsCalibrating}).id == Response::Waiting) {
    wait();
  }
  otos->msg({Command::SelfTest});
  check();
  startBackgroundTasks();
  logger.info("OTOS constructed!");
}

meters_per_second_t OTOS::getVF() const {
  return vf;
}

meters_per_second_t OTOS::getVS() const {
  return vs;
}

radians_per_second_t OTOS::getOmega() const {
  return omega;
}

bool OTOS::check() {
  auto response = otos->msg({Command::SelfTest});
  const bool good{response.correct() && response.id == Response::Success};
  if(!good) {
    logger.error("Detected issue with OTOS.");
  }
  return good;
}

TASK_DEFINITIONS_FOR(OTOS) {
  START_TASK("Update OTOS")
  wait(100_ms);
  while(true) {
    auto vel = otos->msg(Packet{Command::GetVelocity}, 14);
    if(vel.id == Response::Success && vel.correct()) {
      PiecedData<float[3]> velPieced;
      for(int i{0}; i < vel.data.size(); i++) {
        velPieced.bytes[i] = vel.data[i];
      }
      vf = meters_per_second_t{-0.97 * velPieced.value[0]};
      vs = meters_per_second_t{0.97 * velPieced.value[1]};
      omega = radians_per_second_t{-0.9825 * velPieced.value[2]};
    }
    logger.debug("OTOS reading (vf, vs, omega): (" + to_string(vf) + ", " +
                 to_string(vs) + ", " + to_string(omega) + ").");
    wait();
  }
  END_TASK
}
} // namespace atum